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

void plasto_node::local_map_timer(const ros::TimerEvent &)
{
    if (!init_cloud)
        return;

    if (!init_first_pose)
        return;

    t_p_sc mapper_start = system_clock::now();

    pose pose_copy;
    Eigen::Vector3d goal_copy;
    
    get_pose_goal_data(goal_copy, pose_copy);

    local_map_mutex.lock();

    local_cloud = 
        sm.get_sliding_map_from_global(
        pose_copy.pos, pose_copy.rot.q);

    sensor_msgs::PointCloud2 obstacle_msg, detailed_map_msg;
    // Publish local cloud as a ros message
    pcl::toROSMsg(*local_cloud, obstacle_msg);
    
    local_map_mutex.unlock();

    obstacle_msg.header.frame_id = "world";
    obstacle_msg.header.stamp = ros::Time::now();
    local_pcl_pub.publish(obstacle_msg);

    double total = duration<double>(system_clock::now() - mapper_start).count() * 1000;
    std::cout << "mapping duration (" << KGRN <<
        total << "ms" << KNRM << ")" << std::endl;

}

void plasto_node::agent_command_timer(const ros::TimerEvent &)
{
    pose pose_copy;
    Eigen::Vector3d goal_copy;
    
    get_pose_goal_data(goal_copy, pose_copy);

    t_p_sc current_time = system_clock::now();

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    double yaw;

    if (emergency_stop)
    {
        if (duration<double>(system_clock::now() - 
            emergency_stop_time).count() > 0.25)
        {
            publish_or_update_command(
                last_safe_pos, Eigen::Vector3d(), 
                Eigen::Vector3d(), last_safe_yaw);
            return;
        }
        else
        {
            state_mutex.lock();
            emergency_stop = false;
            state = agent_state::PROCESS_MISSION;
            state_mutex.unlock();
            return;
        }
    }

    am_mutex.lock();
    if (am.empty())
    {
        // std::cout << KRED << "[command_timer] " <<
        //     "am empty" << KNRM << std::endl;
        publish_or_update_command(
            last_safe_pos, Eigen::Vector3d(), 
            Eigen::Vector3d(), last_safe_yaw);
        am_mutex.unlock();
        return;
    }

    // Check whether agent is close to the endpoint or
    // the agent is almost completed with the trajectory
    bool stop = false;
    stop = 
        (abs(duration<double>(current_time - 
        am.back().s_e_t.second).count()) < end_time_tol) || 
        ((goal_copy - pose_copy.pos).norm() < end_dist_tol);

    std::lock(state_mutex, last_safe_mutex);
    // If the agent has reached its goal
    if (state != agent_state::IDLE && stop)
    {
        state = agent_state::IDLE;
        is_safe = false;
        double end_time = am.back().traj.getTotalDuration();
        last_safe_pos = am.back().traj.getPos(end_time);
        
        am.clear();
        std::cout << KGRN << "[command_timer] " <<
            "trajectory completed" << KNRM << std::endl;
        pos = last_safe_pos;
        vel = Eigen::Vector3d::Zero();
        acc = Eigen::Vector3d::Zero();

    }
    // If the agent is executing the mission
    else if (state == agent_state::EXEC_MISSION)
    {
        am_trajectory am_segment;
        // Choose the path within the vector of trajectories
        for (am_trajectory &current_am : am)
            if (duration<double>(current_time - 
                current_am.s_e_t.second).count() <= 0.0)
            {
                am_segment = current_am;
                break;
            }

        double t = 
            duration<double>(current_time - 
            am_segment.s_e_t.first).count();
        pos = am_segment.traj.getPos(t);
        // If the agent has not reached its goal
        vel = am_segment.traj.getVel(t);
        acc = am_segment.traj.getAcc(t);
        
        last_safe_pos = pos;
        last_safe_yaw = yaw;
    }
    else
    {
        pos = last_safe_pos;
        yaw = last_safe_yaw;
        vel = Eigen::Vector3d::Zero();
        acc = Eigen::Vector3d::Zero();
    }

    state_mutex.unlock();
    last_safe_mutex.unlock();
    am_mutex.unlock();

    publish_or_update_command(pos, vel, acc, yaw);
    
}

void plasto_node::plasto_timer(const ros::TimerEvent &)
{
    state_mutex.lock();
    if (state == agent_state::IDLE)
    {
        state_mutex.unlock();
        return;
    }
    state_mutex.unlock();

    // printf("start planner\n");

    pose pose_copy;
    Eigen::Vector3d goal_copy;
    
    get_pose_goal_data(goal_copy, pose_copy);

    t_p_sc planner_start = system_clock::now();
    t_p_sc horizon_time = 
        planner_start + milliseconds((int)round(safety_horizon*1000));

    int index;
    t_p_sc found;
    int offset = 0;
    int current_cp_index, current_knot_index;

    AmTraj am_traj(
        a_m_p.w_t, a_m_p.w_a, a_m_p.w_j, 
        a_m_p.m_v, a_m_p.m_a, a_m_p.m_i, a_m_p.e);

    Eigen::Vector3d start_point;
    std::vector<Eigen::Vector3d> check_path, global_search_path;
    int idx = 0;

    std::lock(am_mutex, state_mutex);
    while (1)
    {
        if (am.empty())
            break;

        if (duration<double>(
            planner_start - am.front().s_e_t.second).count() > 0.0)
            am.erase(am.begin());
        else
            break;
    }
    am_mutex.unlock();
    state_mutex.unlock();

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
            start_point = pose_copy.pos;
            check_path.push_back(pose_copy.pos);
            break;
        }
        case (agent_state::EXEC_MISSION):
        {
            am_mutex.lock();
            if (am.empty())
            {
                std::cout << KRED << "[plasto_timer] " <<
                    "am empty found" << KNRM << std::endl;
                am_mutex.unlock();
                state_mutex.unlock();
                return;
            }

            int fail_count = 0;
            // Select point after adding the time horizon
            for (idx = 0; idx < (int)am.size(); idx++)
            {
                if (duration<double>(horizon_time - am[idx].s_e_t.second).count() < 0.0)
                    break;
                else
                    fail_count++;
            }

            if (fail_count == (int)am.size())
            {
                // printf("fail_count == (int)am.size()\n");
                bool finished = 
                    duration<double>(am.back().s_e_t.second - 
                    horizon_time).count() < end_time_tol;
                
                bool too_close = 
                    duration<double>(am.back().s_e_t.second - 
                    horizon_time).count() < safety_horizon / 2;
                
                if (finished || too_close)
                {
                    std::cout << KRED << "[plasto_timer] " <<
                        "finished or too close" << KNRM << std::endl;
                    am.clear();
                    state = agent_state::PROCESS_MISSION;
                    is_safe = false;
                    
                    am_mutex.unlock();
                    state_mutex.unlock();
                    return;
                }
                else
                    horizon_time = am.back().s_e_t.second;
            }

            double t1 = duration<double>(horizon_time - am[idx].s_e_t.first).count();
            start_point = am[idx].traj.getPos(t1);

            check_path = get_am_path_interval(0.4); 

            am_mutex.unlock();
            break;
        }
        default:
        {
            state_mutex.unlock();
            return;
        }
    }
    state_mutex.unlock();

    if ((goal_copy - start_point).norm() < end_dist_tol)
    {
        std::lock(am_mutex, state_mutex);

        am.clear();
        state = agent_state::IDLE;
        is_safe = false;
        
        am_mutex.unlock();
        state_mutex.unlock();
        return;
    }

    local_map_mutex.lock();
    // Update the octree with the local cloud and add in
    // start and goal point for the rrt manager
    rrt.update_pose_and_octree(
        local_cloud, start_point, goal_copy);
    local_map_mutex.unlock();
    // printf("update rrt\n");

    double update_octree_time = duration<double>(system_clock::now() - 
        planner_start).count()*1000;

    /**
     * @brief 
     * Conducting check validity of previous path or to carry 
     * out new rrt search
     */    

    double update_check_time;

    bool replan = !rrt.get_path_validity(check_path) || !is_safe;
    // printf("get_path_validity\n");
    // Check to see whether the previous data extents to the end
    // if previous point last point connects to end point, do bypass    
    if (replan)
    {
        update_check_time = duration<double>(system_clock::now() - 
            planner_start).count()*1000 - update_octree_time;

        global_setpoints_mutex.lock();
        is_safe = rrt.get_path(global_setpoints, sample_tree);

        if (global_setpoints.empty())
        {
            std::lock(am_mutex, state_mutex);

            std::cout << KRED << "[plasto_timer] " <<
                "Collision detected, emergency stop" << KNRM << std::endl;
            
            am.clear();
            emergency_stop = true;
            state = agent_state::IDLE;
            is_safe = false;

            emergency_stop_time = system_clock::now();
            
            am_mutex.unlock();
            state_mutex.unlock();
            global_setpoints_mutex.unlock();
            return;
        }
        global_setpoints_mutex.unlock();

        // if (!is_safe)
        //     std::cout << KRED << "[no global path found] " << KNRM <<
        //         "using [safe path]" << std::endl;

        if (sample_tree)
        {
            visualization_msgs::Marker edges = visualize_line_list(
                rrt.edges,color_range[0], 0.2, 1, 0.5);
            debug_pub.publish(edges);
        }

        if (!local_cloud->points.empty())
        {
            std::shared_ptr<CorridorGen::CorridorGenerator> 
                corridor_generator = 
                std::make_shared<CorridorGen::CorridorGenerator>(
                rrt_param.r, rrt_param.r*2, 50, 
                rrt_param.h_c.second, rrt_param.h_c.first, 
                rrt_param.r*2, true, no_fly_zone);
        
            local_map_mutex.lock();
            corridor_generator->updatePointCloud(local_cloud);
            std::vector<Corridor> corridor_list;
            local_map_mutex.unlock();

            bool corridor_success = 
                corridor_generator->generateCorridorAlongPath(global_setpoints);
            // printf("generateCorridorAlongPath complete \n");
            corridor_list = corridor_generator->getCorridor();
            // printf("getCorridor complete \n");
            visualization_msgs::MarkerArray corridor_msg =
                visualize_corridor(corridor_list, color_range[1]);

            sfc_pub.publish(corridor_msg);

            global_search_path = corridor_generator->getWaypointList();
            // printf("getWaypointList complete \n");
            if (global_search_path.empty())
            {
                std::lock(am_mutex, state_mutex);

                std::cout << KRED << "[plasto_timer] " <<
                    "global_search_path empty, emergency stop" << KNRM << std::endl;
                
                am.clear();
                emergency_stop = true;
                state = agent_state::IDLE;
                is_safe = false;

                emergency_stop_time = system_clock::now();

                state_mutex.unlock();
                am_mutex.unlock();
                return;
            }

            if (!corridor_success)
                is_safe = false;
        }
        else
            global_search_path = global_setpoints;        

        nav_msgs::Path global_path = 
            vector_3d_to_path(global_search_path);
        g_rrt_points_pub.publish(global_path);
    }

    double rrt_time = 
        duration<double>(system_clock::now() - 
        planner_start).count() * 1000;
    
    // std::cout << "total search time(" << KGRN <<
    //     duration<double>(system_clock::now() - 
    //     planner_start).count()*1000 << "ms" << KNRM << 
    //     ") update_octree time(" << local_cloud->points.size() << ") (" << KGRN <<
    //     update_octree_time << "ms" << KNRM << 
    //     ") update_check time(" << KGRN <<
    //     update_check_time << "ms" << KNRM << ")" << std::endl;
    

    /**
     * @brief 
     * Check whether there is a need to update the trajectory by
     * concatenation of the AM trajectory
     */

    std::lock(am_mutex, state_mutex);

    // Update if we are processing the mission
    if (state == agent_state::PROCESS_MISSION)
    {
        am_trajectory tmp_am;
        tmp_am.traj = am_traj.genOptimalTrajDT(
            global_search_path, Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero());
        t_p_sc s_t = system_clock::now();
        tmp_am.s_e_t.first = system_clock::now();
        tmp_am.s_e_t.second = 
            s_t + milliseconds((int)round(
            tmp_am.traj.getTotalDuration()*1000));
        
        am.clear();
        am.push_back(tmp_am);

        state = agent_state::EXEC_MISSION;
    }
    // Only update if we have done a new RRT search
    else if (state == agent_state::EXEC_MISSION && replan)
    {
        // printf("replan\n");
        // If am vector is empty due to other timers removing elements of it
        // do not run this
        if ((int)am.size() < idx || am.empty())
        {
            state_mutex.unlock();
            am_mutex.unlock();
            return;
        }

        // Since we have a new path, the previous trajectory has to shorten its end time
        am[idx].s_e_t.second = horizon_time;
        double get_duration = duration<double>(
            horizon_time - am[idx].s_e_t.first).count();

        // printf("change previous time\n");

        am_trajectory tmp_am;
        tmp_am.traj = am_traj.genOptimalTrajDT(
            global_search_path, am[idx].traj.getVel(get_duration), 
            am[idx].traj.getAcc(get_duration), Eigen::Vector3d::Zero(), 
            Eigen::Vector3d::Zero());
        
        // printf("genOptimalTrajDT\n");
        tmp_am.s_e_t.first = horizon_time;
        tmp_am.s_e_t.second = 
            horizon_time + milliseconds((int)round(
            tmp_am.traj.getTotalDuration()*1000));
        am.push_back(tmp_am);
    }

    std::vector<Eigen::Vector3d> 
        display_am = get_am_path_interval(0.1);

    state_mutex.unlock();
    am_mutex.unlock();
    
    nav_msgs::Path display_am_path = 
        vector_3d_to_path(display_am);
    am_points_pub.publish(display_am_path);

    double am_time = 
        duration<double>(system_clock::now() - 
        planner_start).count() * 1000 - rrt_time;

    double total = duration<double>(system_clock::now() - planner_start).count() * 1000;
    std::cout << "total duration (" << KGRN <<
        total << "ms" << KNRM << ") rrt duration (" << KGRN <<
        rrt_time << "ms" << KNRM << ") am duration (" << KGRN <<
        am_time << "ms" << KNRM << ")" << std::endl;
}