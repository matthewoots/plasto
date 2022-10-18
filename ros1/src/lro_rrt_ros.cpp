/*
* lro_rrt_ros.cpp
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

#include "lro_rrt_ros.h"
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
using namespace lro_rrt_server;

void lro_rrt_ros_node::pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Callback once and save the pointcloud

    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    if (!init_cloud)
    {
        init_cloud = true;
        full_cloud = pcl2_converter(*msg);
        lro_rrt_server::lro_rrt_server_node::parameters map_param = rrt_param;
        map_param.r = m_p.m_r;
        map.set_parameters(map_param);
        map.update_pose_and_octree(full_cloud, current_point, goal);
    }

    return;
}

void lro_rrt_ros_node::command_callback(const geometry_msgs::PointConstPtr& msg)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    geometry_msgs::Point pos = *msg;

    goal = Eigen::Vector3d(pos.x, pos.y, pos.z);

    if (!rrt.initialized())
        rrt.set_parameters(rrt_param);

    global_search_path.clear();
    received_command = true;

    return;
}

/** @brief Construct the search path from RRT search and from its shortened path */
void lro_rrt_ros_node::generate_search_path()
{
    std::cout << "current (" << KBLU << 
        current_point.transpose() << KNRM << ") " << 
        "goal (" << KBLU << 
        goal.transpose() << ") " << KNRM << 
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
bool lro_rrt_ros_node::check_and_update_search(
    Eigen::Vector3d current)
{
    // std::cout << "Conducting check_and_update_search" << std::endl;
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

void lro_rrt_ros_node::local_map_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    // Crop to simulate local sensing
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    // local_cloud = output; // Reset local_cloud

    // Eigen::Vector3d dimension = Eigen::Vector3d(
    //     rrt_param.s_r, rrt_param.s_r, rrt_param.s_r);

    // Eigen::Vector3d min = current_point - dimension;
    // Eigen::Vector3d max = current_point + dimension;

    // pcl::CropBox<pcl::PointXYZ> box_filter;
    // box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
    // box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

    // box_filter.setInputCloud(full_cloud);
    // box_filter.filter(*local_cloud);

    time_point<std::chrono::system_clock> ray_timer = system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_current = raycast_pcl_w_fov(current_point);
    double ray_time = duration<double>(system_clock::now() - ray_timer).count();
    // std::cout << "raycast time (" << KBLU << ray_time * 1000 << KNRM << "ms)" << std::endl;

    if (local_cloud_accumulated.size() > m_p.m_a)
        local_cloud_accumulated.erase(local_cloud_accumulated.begin());
    
    local_cloud_accumulated.push_back(local_cloud_current);

    local_cloud->points.clear();

    for (int i = 0; i < (int)local_cloud_accumulated.size(); i++)
        *local_cloud += *(local_cloud_accumulated[i]);
    
    double ray_n_acc_time = duration<double>(system_clock::now() - ray_timer).count();
    // std::cout << "raycast and accumulation time (" << KBLU << ray_n_acc_time * 1000 << KNRM << "ms)" << std::endl;

    sensor_msgs::PointCloud2 cloud_msg;
    // Publish local cloud as a ros message
    pcl::toROSMsg(*local_cloud, cloud_msg);

    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    local_pcl_pub.publish(cloud_msg);

}

void lro_rrt_ros_node::agent_forward_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    if (received_command && (int)(global_search_path.size()) != 0 && valid)
    {
        // Always use the point after the start point in global search path
        // Eigen::Vector3d displacement = global_search_path[1] - current_point;
        // while (displacement.norm() < agent_step && (int)global_search_path.size() > 1)
        // {
        //     global_search_path.erase(global_search_path.begin()+1);
        //     displacement = global_search_path[1] - current_point;
        // }

        // Eigen::Vector3d direction_vector = displacement / displacement.norm();

        // current_point += direction_vector * agent_step;

        bspline_trajectory::nbs_pva_state_3d state_3d;
        state_3d = b_t.get_nbspline_3d(
            degree, b_p.t_p_v, b_p.c_p_v, system_clock::now(), traj_start_time);
        current_point = state_3d.pos;

        // std::cout << duration<double>(b_p.t_p_v[b_p.t_p_v.size()-1] - system_clock::now()).count() << "s" << std::endl;

        if (global_search_path.size() > 2 && 
            (global_search_path[1] - current_point).norm() < 1.0)
            global_search_path.erase(global_search_path.begin()+1);

        // if ((int)(global_search_path.size()) == 1)
        if (duration<double>(b_p.t_p_v[b_p.t_p_v.size()-1] - system_clock::now()).count() < 0.10)
        {
            global_search_path.clear();
            received_command = false;
            b_p = {};
        }
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_point.x();
    pose.pose.position.y = current_point.y();
    pose.pose.position.z = current_point.z();
    pose_pub.publish(pose);

    visualize_points(0.5, rrt_param.s_r*2);
}

void lro_rrt_ros_node::rrt_search_timer(const ros::TimerEvent &)
{
    if (!received_command)
        return;

    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    time_point<std::chrono::system_clock> timer = system_clock::now();

    bool bypass = false;

    // RRT Search Module

    rrt.update_pose_and_octree(local_cloud, current_point, goal);

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
        // std::cout << KCYN << "Conducting bypass" << KNRM << std::endl;
        update_check_time = duration<double>(system_clock::now() - 
            timer).count()*1000 - update_octree_time;
        bypass = true;
    }
    else
    {
        // std::cout << "Conducting search" << std::endl;
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
        valid = false;
        return;
    }

    // Non-Uniform Bspline
    // Only update if we have done a new RRT search
    if (!bypass)
    {
        // tmp_time_waypoint is the duration from the current time point
        vector<double> tmp_time_waypoint;
        vector<t_p_sc> tmp_time;
        vector<Eigen::Vector3d> tmp_control_points, tmp_global_search_path;
        
        tmp_global_search_path.insert(
            tmp_global_search_path.begin(), global_search_path.begin()+1, global_search_path.end());

        b_t.distribute_3d_control_points(
            degree, global_search_path[0], tmp_global_search_path, max_velocity,
            default_knot_spacing, tmp_time_waypoint, tmp_control_points);
        
        // If there are no previous knots, that means we need to clamp the start
        // We also do not have a time stamp set yet
        if (b_p.c_p_v.empty())
        {
            std::cout << KRED << "No previous control points!" << KNRM << std::endl;
            Eigen::Vector3d s_p = tmp_control_points[0];
            double s_t = tmp_time_waypoint[0];
            double e_t = tmp_time_waypoint[tmp_time_waypoint.size()-1];

            tmp_time_waypoint.insert(
                tmp_time_waypoint.begin(), s_t);
            tmp_time_waypoint.insert(
                tmp_time_waypoint.end(), e_t);

            // Clamping the start point
            for (int i = 0; i < degree; i++)
            {
                tmp_control_points.insert(
                    tmp_control_points.begin(), tmp_control_points[0]);
                tmp_time_waypoint.insert(
                    tmp_time_waypoint.begin(), s_t);
            }

            // Since there is no previous time points to take reference from
            // We start the trajectory time here
            traj_start_time = system_clock::now();

            for (int i = 0; i < (int)tmp_time_waypoint.size(); i++)
            {
                int t_s = (int)round(tmp_time_waypoint[i] * 1000.0);
                tmp_time.push_back(traj_start_time + milliseconds(t_s));
            }
        }
        // Else there are previous knots and control points present
        else
        {
            t_p_sc found;
            int index;
            // Do not update if no nearest knot from the previous vector
            if (!b_t.find_nearest_knot(
                system_clock::now(), b_p.t_p_v, found, index))
            {
                valid = false;
                return;
            }

            // Do not update if there are not enough points before
            if (index - (degree-1) < 0 || index + degree > (b_p.t_p_v.size()-1))
                return;

            // Add previous c_p to beginning of the vector
            // for 3th degree i-2 i-1 i i+1
            for (int i = index - 1; i >= index - (degree-1); i--)
            {
                tmp_control_points.insert(
                    tmp_control_points.begin(), b_p.c_p_v[i]);
            }

            // Add previous t_p to beginning of the vector
            // for 3th degree i-2 i-1 i i+1 i+2 i+3
            for (int i = index + degree - 2; i >= index - (degree-1); i--)
            {
                tmp_time.insert(
                    tmp_time.begin(), b_p.t_p_v[i]);
            } 

            t_p_sc base = b_p.t_p_v[index + degree - 2];

            for (int i = 0; i < (int)tmp_time_waypoint.size(); i++)
            {
                int t_s = (int)round(tmp_time_waypoint[i] * 1000.0);
                tmp_time.push_back(base + milliseconds(t_s));
            }
        }
        
        // Clamping the end point
        for (int i = 0; i < degree; i++)
        {
            int last_index_cp = tmp_control_points.size()-1;
            int last_index_tp = tmp_time.size()-1;
            tmp_control_points.insert(
                tmp_control_points.end(), tmp_control_points[last_index_cp]);
            tmp_time.insert(
                tmp_time.end(), tmp_time[last_index_tp]);
        }

        b_p.c_p_v = tmp_control_points;
        b_p.t_p_v = tmp_time;
    }

    valid = true;
}