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
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    _full_cloud = pcl2_converter(*msg);

    // Crop to simulate local sensing
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    _local_cloud = output; // Reset local_cloud
    sensor_msgs::PointCloud2 cloud_msg;

    Eigen::Vector3d dimension = Eigen::Vector3d(
        rrt_param.s_r, rrt_param.s_r, rrt_param.s_r);

    Eigen::Vector3d min = current_point - dimension;
    Eigen::Vector3d max = current_point + dimension;

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
    box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

    box_filter.setInputCloud(_full_cloud);
    box_filter.filter(*_local_cloud);

    // Publish local cloud as a ros message
    pcl::toROSMsg(*_local_cloud, cloud_msg);

    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();

    local_pcl_pub.publish(cloud_msg);
    return;
}

void tbborrt_ros_node::command_callback(const geometry_msgs::PointConstPtr& msg)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    geometry_msgs::Point pos = *msg;

    rrt_param.s_e.first = current_point;
    rrt_param.s_e.second = Eigen::Vector3d(
        pos.x, pos.y, pos.z);

    rrt.set_parameters(rrt_param, no_fly_zone);

    received_command = true;

    return;
}

/** @brief Construct the search path from RRT search and from its shortened path */
void tbborrt_ros_node::generate_search_path()
{    
    rrt_param.s_e.first = current_point;
    rrt_param.s_e.second = rrt_param.s_e.second;

    std::cout << "rrt_param.s_e.first = " << KBLU << 
        rrt_param.s_e.first.transpose() << " " << KNRM << 
        "rrt_param.s_e.second = " << KBLU << 
        rrt_param.s_e.second.transpose() << " " << KNRM << 
        std::endl;
    // Find a RRT path that is quick and stretches to the end point
    vector<Eigen::Vector3d> path = rrt.find_path(global_search_path);
    
    // Clear global_serach_path
    global_search_path.clear();
    // If path gives an invalid value, execute some form of emergency
    if (path.empty())
    {
        error_counter++;
        std::cout << KBLU << "Error in finding path!" << KNRM << std::endl;
        return;
    }

    // Save global_path
    // Note that global search path includes the current location the agent is at
    global_search_path = path;

}

/** @brief Use this function wisely, since check and update may cause an infinite loop
 * If a bad data is given to the rrt node and it cannot complete the validity check
*/
bool tbborrt_ros_node::check_and_update_search(
    Eigen::Vector3d current)
{
    // If it contains just 1 node which is its current point
    if (global_search_path.size() == 1)
        return false;

    // Check to see whether the new control point and the previous inputs
    // have any pointclouds lying inside
    int last_safe_idx = -1;
    for (int i = 0; i < global_search_path.size()-1; i++)
    {
        if (!rrt.check_line_validity(
            global_search_path[i], global_search_path[i+1]))
        {
            last_safe_idx = i;
            break;
        }
    }

    if (last_safe_idx >= 0)
    {
        for (int i = last_safe_idx + 1; i < global_search_path.size(); i++)
            global_search_path.erase(global_search_path.end());
        return false;
    }
    else
        return true;
}

void tbborrt_ros_node::agent_forward_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    if (received_command && (int)(global_search_path.size()) != 0)
    {
        // Always use the point after the start point in global search path
        Eigen::Vector3d displacement = global_search_path[1] - current_point;
        while (displacement.norm() < agent_step && (int)global_search_path.size() > 1)
        {
            global_search_path.erase(global_search_path.begin()+1);
            displacement = global_search_path[1] - current_point;
        }

        Eigen::Vector3d direction_vector = displacement / displacement.norm();

        current_point += direction_vector * agent_step;
        // std::cout << "current_point [" << KBLU << 
        //     current_point.transpose() << KNRM << "]" << std::endl;

        if ((int)(global_search_path.size()) == 1)
            received_command = false;
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_point.x();
    pose.pose.position.y = current_point.y();
    pose.pose.position.z = current_point.z();
    pose_pub.publish(pose);

    visualize_points(0.5, rrt_param.s_r*2);
}

void tbborrt_ros_node::rrt_search_timer(const ros::TimerEvent &)
{
    if (!received_command)
        return;

    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    time_point<std::chrono::system_clock> timer = system_clock::now();

    rrt.update_octree(_local_cloud, current_point, rrt_param.s_e.second);

    double update_octree_time = duration<double>(system_clock::now() - 
        timer).count()*1000;

    // Update global path with current point
    if (!global_search_path.empty())
        global_search_path.erase(global_search_path.begin());
    
    global_search_path.insert(global_search_path.begin(), current_point);

    double update_check_time;
    // Check to see whether the previous data extents to the end
    // if previous point last point connects to end point, do bypass    
    if (check_and_update_search(current_point))
    {
        update_check_time = duration<double>(system_clock::now() - 
            timer).count()*1000 - update_octree_time;
        std::cout << KCYN << "Conducting Bypass" << KNRM << std::endl;
    }
    else
    {
        update_check_time = duration<double>(system_clock::now() - 
            timer).count()*1000 - update_octree_time;
        generate_search_path();
    }
    
    std::cout << "full search time taken = " << KGRN <<
        duration<double>(system_clock::now() - 
        timer).count()*1000 << "ms" << KNRM << 
        " update_octree time taken = " << KGRN <<
        update_octree_time << "ms" << KNRM << 
        " update_check time taken = " << KGRN <<
        update_check_time << "ms" << KNRM << std::endl;

    if (!global_search_path.empty())
    {
        nav_msgs::Path global_path = vector_3d_to_path(global_search_path);
        g_rrt_points_pub.publish(global_path);
    }
    else
    {
        std::cout << KRED << "no global path found!" << KNRM << std::endl;
        return;
    }

}