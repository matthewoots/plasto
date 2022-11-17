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

    // We have a global cloud received here
    if (!init_cloud && have_global_cloud)
    {
        init_cloud = true;
        full_cloud = pcl2_converter(*msg);

        sm.set_parameters(
            m_p.hfov, m_p.vfov, m_p.m_r, 
            m_p.s_r, m_p.s_m_s,
            full_cloud, false);
    }
    // We have a local cloud received here
    else if (!have_global_cloud)
    {
        init_cloud = true;
        full_cloud = pcl2_converter(*msg);
        local_cloud = sm.get_sliding_map_from_sensor(
            full_cloud, current_pose.pos, current_pose.rot.q);
        sensor_msgs::PointCloud2 obstacle_msg;
        // Publish local cloud as a ros message
        pcl::toROSMsg(*local_cloud, obstacle_msg);

        obstacle_msg.header.frame_id = "world";
        obstacle_msg.header.stamp = ros::Time::now();
        local_pcl_pub.publish(obstacle_msg);
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

    if (state == agent_state::IDLE)
        state = agent_state::PROCESS_MISSION;
    else if (state == agent_state::EXEC_MISSION)
        state = agent_state::SWITCH_MISSION;

    return;
}

void lro_rrt_ros_node::local_map_timer(const ros::TimerEvent &)
{
    if (!init_cloud)
        return;

    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    local_cloud = 
        sm.get_sliding_map_from_global(
        current_pose.pos, current_pose.rot.q);

    sensor_msgs::PointCloud2 obstacle_msg, detailed_map_msg;
    // Publish local cloud as a ros message
    pcl::toROSMsg(*local_cloud, obstacle_msg);

    obstacle_msg.header.frame_id = "world";
    obstacle_msg.header.stamp = ros::Time::now();
    local_pcl_pub.publish(obstacle_msg);

}

void lro_rrt_ros_node::agent_forward_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    Eigen::Vector3d vel, acc;
    if (state == agent_state::EXEC_MISSION && !emergency_stop)
    {
        // Choose the path within the vector of trajectories
        t_p_sc current_time = system_clock::now();
        
        am_trajectory am_segment;
        for (am_trajectory &current_am : am)
            if (duration<double>(current_time - current_am.s_e_t.second).count() <= 0.0)
            {
                am_segment = current_am;
                break;
            }

        double t = duration<double>(current_time - am_segment.s_e_t.first).count();
        current_pose.pos = am_segment.traj.getPos(t);

        // If the agent has reached its goal
        if ((goal - current_pose.pos).norm() < 0.1)
        {
            am.clear();
            state = agent_state::IDLE; 
            is_safe = false;
            printf("trajectory completed\n");
        }
        // If the agent has not reached its goal
        else
        {
            vel = am_segment.traj.getVel(t);
            acc = am_segment.traj.getAcc(t);

            if (vel.norm() > 0.10)
                current_pose.rot.e.z() = atan2(vel.y(), vel.x());
        }
    }

    if (emergency_stop)
    {
        double t = duration<double>(system_clock::now() - emergency_stop_time).count();
        if (t > 1.0)
        {
            emergency_stop = false;
            state = agent_state::PROCESS_MISSION;
        }
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = current_pose.pos.x();
    pose.pose.position.y = current_pose.pos.y();
    pose.pose.position.z = current_pose.pos.z();

    calc_uav_orientation(
        acc, current_pose.rot.e.z(), current_pose.rot.q, current_pose.rot.r);

    pose.pose.orientation.w = current_pose.rot.q.w();
	pose.pose.orientation.x = current_pose.rot.q.x();
	pose.pose.orientation.y = current_pose.rot.q.y();
	pose.pose.orientation.z = current_pose.rot.q.z();

    pose_pub.publish(pose);
}

void lro_rrt_ros_node::rrt_search_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    if (state == agent_state::IDLE)
        return;

    time_point<std::chrono::system_clock> timer = system_clock::now();
    t_p_sc horizon_time = 
        timer + milliseconds((int)round(safety_horizon*1000));

    bool bypass = false;
    int index;
    t_p_sc found;
    int offset = 0;
    int current_cp_index, current_knot_index;

    AmTraj am_traj(
        a_m_p.w_t, a_m_p.w_a, a_m_p.w_j, 
        a_m_p.m_v, a_m_p.m_a, a_m_p.m_i, a_m_p.e);

    // Discard any unused previous trajectories in the vector
    if (!am.empty())
    {
        while (1)
        {
            if (duration<double>(
                timer - am.front().s_e_t.second).count() > 0.0)
                am.erase(am.begin());
            else
                break;
        }
    }

    Eigen::Vector3d start_point;
    std::vector<Eigen::Vector3d> check_path, global_search_path;
    int idx;

    switch (state)
    {
        case (agent_state::PROCESS_MISSION):
        {
            // Update the octree with the local cloud
            rrt.update_pose_and_octree(local_cloud, current_pose.pos, goal);
            start_point = current_pose.pos;
            check_path.push_back(current_pose.pos);
            break;
        }
        case (agent_state::EXEC_MISSION):
        {
            // Select point after adding the time horizon
            for (idx = 0; idx < (int)am.size(); idx++)
                if (duration<double>(horizon_time - am[idx].s_e_t.second).count() < 0.0)
                    break;
            
            Eigen::Vector3d point;

            double t1 = duration<double>(horizon_time - am[idx].s_e_t.first).count();
            if (t1 > duration<double>(
                am[idx].s_e_t.second - am[idx].s_e_t.first).count())
                return;
            
            point = am[idx].traj.getPos(t1);

            // Update the octree with the local cloud
            rrt.update_pose_and_octree(local_cloud, point, goal);
            start_point = point;

            // Find the possible trajectory segment it is at
            // so as to delete the paths before
            int p1 = am[idx].traj.locatePieceIdx(t1);
            check_path.push_back(point);
            for (int i = p1; i < (int)am[idx].traj.pieces.size(); i++)
            {
                double seg_duration = am[idx].traj.pieces[i].getDuration();
                if (duration<double>(am[idx].s_e_t.second - 
                    am[idx].s_e_t.first).count() < seg_duration)
                    break;

                check_path.push_back(
                    am[idx].traj[i].getPos(seg_duration));
            }

            for (int i = idx+1; i < (int)am.size(); i++)
                for (int j = 0; i < (int)am[i].traj.pieces.size(); j++)
                {
                    double seg_duration = am[i].traj.pieces[j].getDuration();
                    if (duration<double>(am[i].s_e_t.second - 
                        am[i].s_e_t.first).count() < seg_duration)
                        break;
                    
                    check_path.push_back(
                        am[i].traj.pieces[j].getPos(seg_duration));
                }
            
            break;
        }
        case (agent_state::SWITCH_MISSION):
        {
            // Select point after adding the time horizon
            for (idx = 0; idx < (int)am.size(); idx++)
                if (duration<double>(horizon_time - am[idx].s_e_t.second).count() < 0.0)
                    break;
            
            Eigen::Vector3d point;

            double t1 = duration<double>(horizon_time - am[idx].s_e_t.first).count();
            if (t1 > duration<double>(
                am[idx].s_e_t.second - am[idx].s_e_t.first).count())
                return;
            
            point = am[idx].traj.getPos(t1);

            // Update the octree with the local cloud
            rrt.update_pose_and_octree(local_cloud, point, goal);
            start_point = point;
            
            state = agent_state::EXEC_MISSION;
            is_safe = false;
            check_path.push_back(point);
            
            break;
        }
        default:
            return;
    }

    double update_octree_time = duration<double>(system_clock::now() - 
        timer).count()*1000;

    double update_check_time;
    // Check to see whether the previous data extents to the end
    // if previous point last point connects to end point, do bypass    
    if ((rrt.get_path_validity(check_path) && is_safe) || 
        (goal - start_point).norm() < 0.1)
    {
        // std::cout << KCYN << "Conducting bypass" << KNRM << std::endl;
        update_check_time = duration<double>(system_clock::now() - 
            timer).count()*1000 - update_octree_time;
        bypass = true;
    }
    else
    {
        update_check_time = duration<double>(system_clock::now() - 
            timer).count()*1000 - update_octree_time;

        global_search_path.clear();
        std::vector<Eigen::Vector3d> t_g_s_p;
        is_safe = rrt.get_path(t_g_s_p, sample_tree);

        if (t_g_s_p.empty())
        {
            std::cout << KRED << "Collision detected, emergency stop" << 
                KNRM << std::endl;
            emergency_stop = true;
            am.clear();
            state = agent_state::IDLE;
            emergency_stop_time = system_clock::now();
            return;
        }

        lro_rrt_server::get_discretized_path(t_g_s_p, global_search_path);
        // for (Eigen::Vector3d &p : global_search_path)
        //     std::cout << p.transpose() << std::endl;

        if (!is_safe)
            std::cout << KRED << "[no global path found] " << KNRM <<
                "using [safe path]" << std::endl;
        
        nav_msgs::Path global_path = 
            vector_3d_to_path(global_search_path);
        g_rrt_points_pub.publish(global_path);

        if (sample_tree)
        {
            visualization_msgs::Marker edges = visualize_line_list(
                rrt.edges,color_range[0], 0.2, 1, 0.5);
            debug_pub.publish(edges);
        }
    }
    
    std::cout << "total search time(" << KGRN <<
        duration<double>(system_clock::now() - 
        timer).count()*1000 << "ms" << KNRM << 
        ") update_octree time(" << local_cloud->points.size() << ") (" << KGRN <<
        update_octree_time << "ms" << KNRM << 
        ") update_check time(" << KGRN <<
        update_check_time << "ms" << KNRM << ")" << std::endl;
    
    // Only update if we have done a new RRT search
    if (!bypass && state == agent_state::PROCESS_MISSION)
    {
        am_trajectory tmp_am;
        tmp_am.traj = am_traj.genOptimalTrajDTC(
            global_search_path, Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero());
        t_p_sc s_t = system_clock::now();
        tmp_am.s_e_t.first = system_clock::now();
        tmp_am.s_e_t.second = 
            s_t + milliseconds((int)round(
            tmp_am.traj.getTotalDuration()*1000));

        am.push_back(tmp_am);

        state = agent_state::EXEC_MISSION;
    }
    else if (!bypass && state == agent_state::EXEC_MISSION)
    {
        // Since we have a new path, the previous trajectory has to shorten its end time
        am[idx].s_e_t.second = horizon_time;
        double get_duration = duration<double>(
            horizon_time - am[idx].s_e_t.first).count();

        am_trajectory tmp_am;
        tmp_am.traj = am_traj.genOptimalTrajDTC(
            global_search_path, am[idx].traj.getVel(get_duration), 
            am[idx].traj.getAcc(get_duration), Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero());
        tmp_am.s_e_t.first = horizon_time;
        tmp_am.s_e_t.second = 
            horizon_time + milliseconds((int)round(
            tmp_am.traj.getTotalDuration()*1000));
    
        am.push_back(tmp_am);
    }
}

void lro_rrt_ros_node::calc_uav_orientation(
	Eigen::Vector3d acc, double yaw_rad, Eigen::Quaterniond &q, Eigen::Matrix3d &r)
{
	Eigen::Vector3d alpha = acc + Eigen::Vector3d(0,0,9.81);
	Eigen::Vector3d xC(cos(yaw_rad), sin(yaw_rad), 0);
	Eigen::Vector3d yC(-sin(yaw_rad), cos(yaw_rad), 0);
	Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
	Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
	Eigen::Vector3d zB = xB.cross(yB);

	Eigen::Matrix3d R;
	R.col(0) = xB;
	R.col(1) = yB;
	R.col(2) = zB;

    r = R;

	Eigen::Quaterniond q_tmp(R);
    q = q_tmp;

	return;
}