/*
* callbacks.cpp
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

#include "plasto.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

void plasto_node::pose_callback(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
    Eigen::Affine3d transform;

    // local position in frame
	transform.translation() = Vector3d(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z
	);
	// local rotation in frame
	transform.linear() = Quaterniond(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z).toRotationMatrix();

    if (is_enu)
    {
        // Conversion (rotation) from enu to nwu in the form of w, x, y, z
        // enu to nwu
        transform.rotate(
            Quaterniond(0.7073883, 0, 0, 0.7068252).inverse());
    }

    pose_update_mutex.lock();

    current_pose.pos = transform.translation();
    current_pose.rot.q = transform.linear();
    current_pose.rot.e.z() = 
        euler_rpy(transform.linear()).z();

    init_first_pose = true;
    
    pose_update_mutex.unlock();
}

void plasto_node::pointcloud_callback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Callback once and save the pointcloud

    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    pose pose_copy;
    Eigen::Vector3d goal_copy;
    
    get_pose_goal_data(
        goal_copy, pose_copy);

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

        local_map_mutex.lock();
        
        local_cloud = sm.get_sliding_map_from_sensor(
            full_cloud, pose_copy.pos, pose_copy.rot.q);
        
        local_map_mutex.unlock();

        sensor_msgs::PointCloud2 obstacle_msg;
        // Publish local cloud as a ros message
        pcl::toROSMsg(*local_cloud, obstacle_msg);

        obstacle_msg.header.frame_id = "world";
        obstacle_msg.header.stamp = ros::Time::now();
        local_pcl_pub.publish(obstacle_msg);
    }

    return;
}

void plasto_node::command_callback(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    geometry_msgs::PoseStamped pos = *msg;

    state_mutex.lock();
    last_safe_mutex.lock();
    am_mutex.lock();
    goal_update_mutex.lock();
    pose_update_mutex.lock();

    pose pose_copy = current_pose;

    goal = Eigen::Vector3d(
        pos.pose.position.x, 
        pos.pose.position.y, 
        pos.pose.position.z
    );   

    bool not_at_end = false;
    if (!am.empty())
        not_at_end = (abs(duration<double>(system_clock::now() - 
            am.back().s_e_t.second).count()) >= end_time_tol);

    if (state == agent_state::EXEC_MISSION && not_at_end)
        state = agent_state::EXEC_MISSION;
    else
    {
        am.empty();
        last_safe_pos = pose_copy.pos;
        last_safe_yaw = pose_copy.rot.e.z();
        state = agent_state::PROCESS_MISSION;
    }

    is_safe = false;

    state_mutex.unlock();
    last_safe_mutex.unlock();
    am_mutex.unlock();
    goal_update_mutex.unlock();
    pose_update_mutex.unlock();

    printf("[plasto] received command\n");

    return;
}