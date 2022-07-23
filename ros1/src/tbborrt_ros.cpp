/*
* ttborrt_ros.cpp
*
* ---------------------------------------------------------------------
* Copyright (C) 2022 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include "tbborrt_ros.h"
#include <pcl/filters/crop_box.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace std;
using namespace Eigen;
using namespace tbborrt_server;

void tbborrt_ros_node::pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex);
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    _full_cloud = pcl2_converter(*msg);

    // Crop to simulate local sensing
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    _local_cloud = output; // Reset local_cloud
    sensor_msgs::PointCloud2 cloud_msg;

    Eigen::Vector3d dimension = Eigen::Vector3d(
        _sensor_range, _sensor_range, _sensor_range);

    Eigen::Vector3d min = start - dimension;
    Eigen::Vector3d max = start + dimension;

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
    box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

    box_filter.setInputCloud(_full_cloud);
    box_filter.filter(*_local_cloud);

    last_pcl_msg = ros::Time::now();

    // Publish local cloud as a ros message
    // pcl::toROSMsg(*_local_cloud, cloud_msg);

    // cloud_msg.header.frame_id = "world";
    // cloud_msg.header.stamp = ros::Time::now();

    // local_pcl_pub.publish(cloud_msg);
    return;
}

/** @brief Construct the search path from RRT search and from its shortened path */
void tbborrt_ros_node::generate_search_path()
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    start_end.first = current_point;

    // Find a RRT path that is quick and stretches to the end point
    vector<Eigen::Vector3d> path = rrt.find_path(previous_search_points, start_end);
    
    // If path gives an invalid value, execute some form of emergency
    if (path.empty())
    {
        return;
    }

    // Save global_path
    global_search_path.clear();
    global_search_path = path;
}

/** @brief Use this function wisely, since check and update may cause an infinite loop
 * If a bad data is given to the rrt node and it cannot complete the validity check
*/
bool tbborrt_ros_node::check_and_update_search(
    Eigen::Vector3d current)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);
    std::lock_guard<std::mutex> search_points_lock(search_points_mutex);

    if (previous_search_points.empty())
        return false;

    // Copy previous search points
    vector<Eigen::Vector3d> temporary_previous_points = previous_search_points;
    temporary_previous_points.insert(
        temporary_previous_points.begin(), current);

    // Check to see whether the new control point and the previous inputs
    // have any pointclouds lying inside
    int last_safe_idx = -1;
    for (int i = 0; i < temporary_previous_points.size()-1; i++)
    {
        if (!rrt.check_line_validity(
            previous_search_points[i], previous_search_points[i+1]))
        {
            last_safe_idx = i;
            break;
        }
    }

    if (last_safe_idx >= 0)
    {
        for (int i = last_safe_idx + 1; i < temporary_previous_points.size(); i++)
            previous_search_points.erase(previous_search_points.end());
        return false;
    }
    else
        return true;
}

void tbborrt_ros_node::run_search_timer(const ros::TimerEvent &)
{
    if ((ros::Time::now() - last_pcl_msg).toSec() > 2.0)
        return;

    std::lock_guard<std::mutex> cloud_lock(cloud_mutex);

    /** @brief Start of RRT search process **/

    rrt.update_octree(_local_cloud);
    rrt.set_parameters(_obstacle_threshold, _no_fly_zone, 
            _runtime_error, _height_constrain, _sensor_range, _resolution);

    // Check to see whether the previous data extents to the end
    // if previous point last point connects to end point, do bypass    
    if (!check_and_update_search(current_point))
    {
        // Clear global path so that current point can be added
        global_search_path.clear();
        global_search_path.push_back(current_point);
        for (int i = 0; i < previous_search_points.size(); i++)
        {
            global_search_path.push_back(previous_search_points[i]);
        }
    }
    // Check to see whether the previous data extents to the end
    // If previous point last point does not connect to end point, there is no bypass
    else
        generate_search_path();
    
    nav_msgs::Path global_path = vector_3d_to_path(global_search_path);
    g_rrt_points_pub.publish(global_path);

    /** @brief End of RRT search process **/


    // Project the point forward and find vector of travel
    double step_size = 0.5;
    bool delete_vector = false;
    Eigen::Vector3d global_vector = Eigen::Vector3d::Zero();
    for (int i = 1; i < global_search_path.size(); i++)
    {
        Eigen::Vector3d vect2 = global_search_path[i] - current_point;
        Eigen::Vector3d vect2_dir = vect2 / vect2.norm();

        if (vect2.norm() < step_size * 0.9)
        {
            delete_vector = true;
            continue;
        }

        global_vector = vect2_dir;
        break;
    }
    if (delete_vector)
        previous_search_points.erase(previous_search_points.begin());

    current_point += global_vector * step_size;
    std::cout << "current_point [" << KBLU << 
        current_point.transpose() << "]" << KNRM << std::endl;
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_point.x();
    pose.pose.position.y = current_point.y();
    pose.pose.position.z = current_point.z();
    pose_pub.publish(pose);

    visualize_points(0.5, _sensor_range*2);
    std::cout << std::endl;
}