/*
* plasto.cpp
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

    if (!init_last_pose)
    {
        last_goal_pos = current_pose.pos;
        init_last_pose = true;
    }

    get_first_pose = true;

    pose_update_mutex.unlock();
}

void plasto_node::pcl2_callback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Callback once and save the pointcloud

    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    pose_update_mutex.lock();
    pose pose_copy = current_pose;
    pose_update_mutex.unlock();

    // We have a global cloud received here
    if (!init_cloud && have_global_cloud)
    {
        init_cloud = true;
        full_cloud = pcl2_converter(*msg);

        sm.set_parameters(
            m_p.hfov, m_p.vfov, m_p.m_r, 
            m_p.s_r, m_p.s_m_s,
            full_cloud, false, distance_threshold);
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

    pose_update_mutex.lock();
    pose pose_copy = current_pose;
    pose_update_mutex.unlock();

    goal_update_mutex.lock();

    goal = Eigen::Vector3d(
        pos.pose.position.x, 
        pos.pose.position.y, 
        pos.pose.position.z
    );

    goal_update_mutex.unlock();

    if (!rrt.initialized())
        rrt.set_parameters(rrt_param);

    state_mutex.lock();
    
    if (state == agent_state::IDLE)
        state = agent_state::PROCESS_MISSION;
    // else if (state == agent_state::EXEC_MISSION)
    //     state = agent_state::SWITCH_MISSION;
    else if (state == agent_state::EXEC_MISSION)
    {
        am.clear();
        state = agent_state::PROCESS_MISSION;
        last_goal_pos = pose_copy.pos;
        is_safe = false;
    }
    
    state_mutex.unlock();

    // Every time a command is executed, 
    // save the current state (using the callback) as fallback
    init_last_pose = false;

    printf("[plasto] received command\n");

    return;
}

void plasto_node::local_map_timer(const ros::TimerEvent &)
{
    if (!init_cloud)
        return;

    if (!get_first_pose)
        return;

    // t_p_sc mapper_start = system_clock::now();

    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);
    pose_update_mutex.lock();
    pose pose_copy = current_pose;
    pose_update_mutex.unlock();

    local_map_mutex.lock();

    local_cloud = 
        sm.get_sliding_map_from_global(
        pose_copy.pos, pose_copy.rot.q);

    local_map_mutex.unlock();

    sensor_msgs::PointCloud2 obstacle_msg, detailed_map_msg;
    // Publish local cloud as a ros message
    pcl::toROSMsg(*local_cloud, obstacle_msg);

    obstacle_msg.header.frame_id = "world";
    obstacle_msg.header.stamp = ros::Time::now();
    local_pcl_pub.publish(obstacle_msg);

    // double total = duration<double>(system_clock::now() - mapper_start).count() * 1000;
    // std::cout << "mapping duration (" << KGRN <<
    //     total << "ms" << KNRM << ")" << std::endl;

}

void plasto_node::agent_command_timer(const ros::TimerEvent &)
{
    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    pose_update_mutex.lock();
    pose pose_copy = current_pose;
    pose_update_mutex.unlock();

    double yaw;

    if (emergency_stop)
    {
        last_goal_pos = pose_copy.pos;
        double t = duration<double>(system_clock::now() - emergency_stop_time).count();
        if (t > 0.25)
        {
            state_mutex.lock();

            emergency_stop = false;
            state = agent_state::PROCESS_MISSION;
            init_last_pose = true;
            
            state_mutex.unlock();
            return;
        }
    }

    am_mutex.lock();

    bool stop = false;
    if (!am.empty())
    {
        stop = 
            abs(duration<double>(system_clock::now() - 
            am.back().s_e_t.second).count()) < rrt_param.s_i*1.0;
        // printf("size %d, %lf\n", (int)am.size(),
        //     duration<double>(system_clock::now() - 
        //     am.back().s_e_t.second).count());
    }

    // If the agent has reached its goal
    if ((state != agent_state::IDLE && 
        (goal - pose_copy.pos).norm() < 0.10) || stop)
    {
        am.clear();
        state = agent_state::IDLE;
        last_goal_pos = pos;
        is_safe = false;
        printf("trajectory completed\n");
    }

    am_mutex.unlock();

    state_mutex.lock();

    if (state == agent_state::EXEC_MISSION || 
        state == agent_state::SWITCH_MISSION)
    {
        // Choose the path within the vector of trajectories
        t_p_sc current_time = system_clock::now();
        
        am_mutex.lock();

        if (am.empty())
        {
            state_mutex.unlock();
            am_mutex.unlock();
            return;
        }

        am_trajectory am_segment;
        for (am_trajectory &current_am : am)
            if (duration<double>(current_time - current_am.s_e_t.second).count() <= 0.0)
            {
                am_segment = current_am;
                break;
            }

        double t = duration<double>(current_time - am_segment.s_e_t.first).count();
        pos = am_segment.traj.getPos(t);
        
        // If the agent has not reached its goal
        vel = am_segment.traj.getVel(t);
        acc = am_segment.traj.getAcc(t);
        
        am_mutex.unlock();

        double dist = -FLT_MAX;
        Eigen::Vector3d intersection_point;
        bool intersect_found = false;
        double offset = 0.0;

        global_setpoints_mutex.lock();

        if (!sfc_polygon.empty() && 
            (goal - pose_copy.pos).norm() > 0.75)
            for (int i = 0; i < global_setpoints.size()-1; i++)
            {
                if (i > 0)
                    offset += 
                        (global_setpoints[i-1] - 
                        global_setpoints[i]).norm();
                
                for (auto &sfc : sfc_polygon)
                {
                    // This is a tricky scenario since the line may intersect the polygon 2 times
                    // Get the furthest from origin
                    for (auto &tri : sfc.tri_idx)
                    {
                        Eigen::Vector3d intersection_point_tmp;
                        double distance_tmp;
                        if (rayTriangleIntersect(
                            global_setpoints[i], global_setpoints[i+1],
                            sfc.vert[tri[0]], sfc.vert[tri[1]], sfc.vert[tri[2]],
                            distance_tmp, intersection_point_tmp))
                        {
                            if (offset + distance_tmp > dist)
                            {
                                intersection_point = intersection_point_tmp;
                                dist = offset + distance_tmp;
                            }
                            intersect_found = true;
                        }
                    }
                }
            }
        
        global_setpoints_mutex.unlock();
        
        if (intersect_found)
        {
            Eigen::Vector3d yaw_direction = 
                (intersection_point - pose_copy.pos).normalized();
            yaw = atan2(yaw_direction.y(), yaw_direction.x());
            last_yaw = yaw;
            // printf("intersect yaw %lf\n", yaw);
        }
        else
        {
            yaw = last_yaw;
            // printf("no intersect yaw %lf\n", yaw);
        }
        
    }
    else
    {
        pos = last_goal_pos;
        yaw = pose_copy.rot.e.z();
    }

    state_mutex.unlock();
    
    if (simulation)
    {
        pose_update_mutex.lock();

        current_pose.pos = pos;
        current_pose.rot.e.z() = yaw;
        
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

        last_goal_pos = pos;
        get_first_pose = true;

        pose_update_mutex.unlock();
    }
    // Not in a simulation
    else if (!simulation)
    // else if (!simulation && state != agent_state::IDLE)
    {
        mavros_msgs::PositionTarget target;
        target.position.x = pos.x();
        target.position.y = pos.y();
        target.position.z = pos.z();

        target.velocity.x = vel.x();
        target.velocity.y = vel.y();
        target.velocity.z = vel.z();

        target.acceleration_or_force.x = acc.x();
        target.acceleration_or_force.y = acc.y();
        target.acceleration_or_force.z = acc.z();

        target.yaw = yaw;

        command_pub.publish(target);
    }
}

void plasto_node::plasto_timer(const ros::TimerEvent &)
{
    // std::lock_guard<std::mutex> pose_lock(pose_update_mutex);

    if (state == agent_state::IDLE)
        return;

    if (!init_last_pose)
        return;

    pose_update_mutex.lock();
    goal_update_mutex.lock();

    pose pose_copy = current_pose;
    Eigen::Vector3d goal_copy = goal;

    goal_update_mutex.unlock();
    pose_update_mutex.unlock();

    t_p_sc planner_start = system_clock::now();

    std::vector<octree_map::sliding_map::triangles> tri_vector =
        sm.visualize_safe_corridor();
    
    sfc_polygon.clear();
    sfc_polygon = tri_vector;

    visualization_msgs::Marker safe_corridor = 
        visualize_triangle_list(color_range[2], tri_vector);

    safe_corridor_pub.publish(safe_corridor);

    double sfc_time = 
        duration<double>(system_clock::now() - planner_start).count() * 1000;

    t_p_sc timer = system_clock::now();
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

    am_mutex.lock();
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
    am_mutex.unlock();

    Eigen::Vector3d start_point;
    std::vector<Eigen::Vector3d> check_path, global_search_path;
    int idx = 0;

    /**
     * @brief 
     * Conduct an update to the octree by providing the local points
     * in the cloud, and also finding a position [determined by safety_horizon]
     * to use as start
     */

    state_mutex.lock();

    switch (state)
    {
        case (agent_state::PROCESS_MISSION):
        {
            local_map_mutex.lock();

            // Update the octree with the local cloud
            rrt.update_pose_and_octree(
                local_cloud, pose_copy.pos, goal_copy);

            local_map_mutex.unlock();

            start_point = pose_copy.pos;
            check_path.push_back(pose_copy.pos);
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
            {
                state_mutex.unlock();
                return;
            }
            // double time = duration<double>(
            //     horizon_time - am[idx].s_e_t.second).count();
            // printf("size of am %d\n", (int)am.size());    
            // printf("given time of %d %lf\n", 
            //     idx, time);

            point = am[idx].traj.getPos(t1);

            local_map_mutex.lock();

            // Update the octree with the local cloud
            rrt.update_pose_and_octree(
                local_cloud, point, goal_copy);
            
            local_map_mutex.unlock();

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
            am_mutex.lock();
            Eigen::Vector3d point;
            printf("empty am %s\n", am.empty() ? "yes" : "no");

            // printf("switching here\n");
            // Select point after adding the time horizon
            if (!am.empty())
            {
                printf("size of am %d\n", (int)am.size());
                for (idx = 0; idx < (int)am.size(); idx++)
                    if (duration<double>(horizon_time - 
                        am[idx].s_e_t.second).count() < 0.0)
                        break;
                
                idx = min((int)am.size()-1, idx);

                double time = duration<double>(
                    horizon_time - am[idx].s_e_t.second).count();
                printf("given time of %d %lf\n", 
                    idx, time);

                double t1 = duration<double>(horizon_time - am[idx].s_e_t.first).count();
                // if (t1 > duration<double>(
                //     am[idx].s_e_t.second - am[idx].s_e_t.first).count())
                //     return;
                // The horizon time is outside of the last time point of the trajectory
                if (duration<double>(
                    horizon_time - am[idx].s_e_t.second).count() > 0 
                    && idx == am.size()-1)
                {
                    // printf("out of scope\n");
                    horizon_time = am[idx].s_e_t.second - 
                        milliseconds(5);
                    t1 = duration<double>(
                        horizon_time - am[idx].s_e_t.first).count();
                }
                
                point = am[idx].traj.getPos(t1);

                state = agent_state::EXEC_MISSION;
                
            }
            else
            {
                am.clear();
                point = pose_copy.pos;
                state = agent_state::PROCESS_MISSION;
            }

            printf("Updated pose\n");

            am_mutex.unlock();

            local_map_mutex.lock();

            // Update the octree with the local cloud
            rrt.update_pose_and_octree(
                local_cloud, point, goal_copy);

            local_map_mutex.unlock();

            start_point = point;
            
            is_safe = false;
            check_path.push_back(point);
            
            break;
        }
        default:
        {
            state_mutex.unlock();
            return;
        }
    }

    state_mutex.unlock();

    double update_octree_time = duration<double>(system_clock::now() - 
        timer).count()*1000;

    /**
     * @brief 
     * Conducting check validity of previous path or to carry 
     * out new rrt search
     */

    global_setpoints_mutex.lock();

    double update_check_time;
    // Check to see whether the previous data extents to the end
    // if previous point last point connects to end point, do bypass    
    if ((rrt.get_path_validity(check_path) && is_safe) || 
        (goal_copy - start_point).norm() < 0.1)
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

        is_safe = rrt.get_path(global_setpoints, sample_tree);

        if (global_setpoints.empty())
        {
            am_mutex.lock();

            std::cout << KRED << "Collision detected, emergency stop" << 
                KNRM << std::endl;
            emergency_stop = true;
            am.clear();
            state = agent_state::IDLE;
            init_last_pose = false;
            emergency_stop_time = system_clock::now();
            
            am_mutex.unlock();
            global_setpoints_mutex.unlock();
            return;
        }

        // lro_rrt_server::get_discretized_path(
        //     global_setpoints, global_search_path);
        
        // for (Eigen::Vector3d &p : global_search_path)
        //     std::cout << p.transpose() << std::endl;

        if (!is_safe)
            std::cout << KRED << "[no global path found] " << KNRM <<
                "using [safe path]" << std::endl;
        
        // nav_msgs::Path global_path = 
        //     vector_3d_to_path(global_search_path);
        // g_rrt_points_pub.publish(global_path);

        if (sample_tree)
        {
            visualization_msgs::Marker edges = visualize_line_list(
                rrt.edges,color_range[0], 0.2, 1, 0.5);
            debug_pub.publish(edges);
        }

        std::shared_ptr<CorridorGen::CorridorGenerator> 
            corridor_generator = 
            std::make_shared<CorridorGen::CorridorGenerator>(
            rrt_param.r, rrt_param.r*2, 50, 
            rrt_param.h_c.second, rrt_param.h_c.first, 
            rrt_param.r*2, no_fly_zone);
    
        local_map_mutex.lock();

        corridor_generator->updatePointCloud(local_cloud);
        std::vector<Corridor> corridor_list;

        std::vector<Eigen::Vector3d> discrete_path;
        lro_rrt_server::get_discretized_path(
            global_setpoints, discrete_path);

        corridor_generator->generateCorridorAlongPath(discrete_path);
        corridor_list = corridor_generator->getCorridor();
        visualization_msgs::MarkerArray corridor_msg =
            visualize_corridor(corridor_list, color_range[1]);

        sfc_pub.publish(corridor_msg);

        local_map_mutex.unlock();

        global_search_path = corridor_generator->getWaypointList();

        if (global_search_path.empty() || 
            global_search_path.size() < 2)
        {
            am_mutex.lock();

            std::cout << KRED << "global_search_path empty, emergency stop" << 
                KNRM << std::endl;
            emergency_stop = true;
            am.clear();
            state = agent_state::IDLE;
            init_last_pose = false;
            emergency_stop_time = system_clock::now();

            am_mutex.unlock();
            global_setpoints_mutex.unlock();
            return;
        }

        nav_msgs::Path global_path = 
            vector_3d_to_path(global_search_path);
        g_rrt_points_pub.publish(global_path);
    }

    // rrt.get_receding_path(
    //     start_point, m_p.s_r * 2, global_search_path);

    // lro_rrt_server::get_discretized_path(
    //     global_setpoints, global_search_path);
    
    global_setpoints_mutex.unlock();

    double rrt_time = 
        duration<double>(system_clock::now() - 
        planner_start).count() * 1000 - sfc_time;
    
    // std::cout << "total search time(" << KGRN <<
    //     duration<double>(system_clock::now() - 
    //     timer).count()*1000 << "ms" << KNRM << 
    //     ") update_octree time(" << local_cloud->points.size() << ") (" << KGRN <<
    //     update_octree_time << "ms" << KNRM << 
    //     ") update_check time(" << KGRN <<
    //     update_check_time << "ms" << KNRM << ")" << std::endl;
    

    /**
     * @brief 
     * Check whether there is a need to update the trajectory by
     * concatenation of the AM trajectory
     */

    am_mutex.lock();
    state_mutex.lock();

    // Update if we are processing the mission
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

        std::vector<Eigen::Vector3d> display_am;
        for (int i = 0; i < (int)am.size(); i++)
        {
            double total_duration = 
                duration<double>(am[i].s_e_t.second - 
                am[i].s_e_t.first).count();
            
            double split = ceil(total_duration / 0.1); 
            double interval = total_duration / split;
            printf("interval %lfl\n",interval);
            double accumulated = 0.0;
            for (int k = 0; k < (int)split; k++)
            {
                if (accumulated > total_duration)
                    continue;
                display_am.push_back(
                    am[i].traj.getPos(accumulated));
                accumulated += interval;
            }

        }
        
        nav_msgs::Path display_am_path = 
            vector_3d_to_path(display_am);
        am_points_pub.publish(display_am_path);

        state = agent_state::EXEC_MISSION;
    }
    // Only update if we have done a new RRT search
    else if (!bypass && 
        state == agent_state::EXEC_MISSION || 
        state == agent_state::SWITCH_MISSION)
    // else if (state == agent_state::EXEC_MISSION)
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

        std::vector<Eigen::Vector3d> display_am;
        for (int i = 0; i < (int)am.size(); i++)
        {
            double total_duration = 
                duration<double>(am[i].s_e_t.second - 
                am[i].s_e_t.first).count();
            
            double split = ceil(total_duration / 0.1); 
            double interval = total_duration / split;
            printf("interval %lfl\n",interval);
            double accumulated = 0.0;
            for (int k = 0; k < (int)split; k++)
            {
                if (accumulated > total_duration)
                    continue;
                display_am.push_back(
                    am[i].traj.getPos(accumulated));
                accumulated += interval;
            }

        }
        
        nav_msgs::Path display_am_path = 
            vector_3d_to_path(display_am);
        am_points_pub.publish(display_am_path);
    }

    state_mutex.unlock();
    am_mutex.unlock();

    double am_time = 
        duration<double>(system_clock::now() - 
        planner_start).count() * 1000 - rrt_time - sfc_time;

    double total = duration<double>(system_clock::now() - planner_start).count() * 1000;
    std::cout << "total duration (" << KGRN <<
        total << "ms" << KNRM << ") sfc duration (" << KGRN <<
        sfc_time << "ms" << KNRM << ") rrt duration (" << KGRN <<
        rrt_time << "ms" << KNRM << ") am duration (" << KGRN <<
        am_time << "ms" << KNRM << ")" << std::endl;
}

void plasto_node::calc_uav_orientation(
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