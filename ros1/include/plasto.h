/*
* plasto.h
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
#ifndef PLASTO_ROS_H
#define PLASTO_ROS_H

#include "lro_rrt_server.h"
#include "mapper.h"
#include "am_traj.hpp"
#include "corridor_gen.h"

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mavros_msgs/PositionTarget.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

// using namespace Eigen;
using namespace std;
using namespace std::chrono; // nanoseconds, system_clock, seconds
using namespace lro_rrt_server;
using namespace octree_map;
using CorridorGen::Corridor;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

class plasto_node
{
    private:

        struct map_parameters
        {
            double m_r; // map resolution
            double s_r;
            double vfov;
            double hfov;
            double s_m_s; // sliding map size
        };

        struct am_trajectory_parameters
        {
            double w_t; // weight for the time regularization
            double w_a; // weight for the integrated squared norm of acceleration
            double w_j; // weight for the integrated squared norm of jerk
            double m_v; // maximum velocity rate
            double m_a; // maximum acceleration rate
            int m_i; // maximum number of iterations in optimization
            double e; // relative tolerance
           
        };

        struct am_trajectory
        {
            std::pair<t_p_sc, t_p_sc> s_e_t; // start and end time of the trajectory
            Trajectory traj;
        };

        struct orientation
        {
            Eigen::Vector3d e; // euler angles
            Eigen::Quaterniond q; // quaternion
            Eigen::Matrix3d r; // rotation matrix
        };

        struct pose
        {
            Eigen::Vector3d pos;
            orientation rot;
        };

        enum agent_state
        {
            IDLE,
            PROCESS_MISSION,
            EXEC_MISSION
        };

        lro_rrt_server::lro_rrt_server_node rrt;
        lro_rrt_server::parameters rrt_param;
        map_parameters m_p;

        octree_map::sliding_map sm;

        double map_size;

        am_trajectory_parameters a_m_p; // am trajectory parameters
        std::vector<am_trajectory> am;

        std::mutex pose_update_mutex;
        std::mutex goal_update_mutex;
        std::mutex local_map_mutex;
        std::mutex am_mutex;
        std::mutex global_setpoints_mutex;
        std::mutex state_mutex;
        std::mutex last_safe_mutex;

        ros::NodeHandle _nh;

        ros::Subscriber pcl2_msg_sub;
        ros::Subscriber command_sub;
        ros::Subscriber pose_sub;
        
        ros::Publisher local_pcl_pub;
        ros::Publisher g_rrt_points_pub; 
        ros::Publisher command_pub;
        ros::Publisher pose_pub;
        ros::Publisher debug_pcl_pub;
        ros::Publisher debug_pub;
        ros::Publisher safe_corridor_pub;
        ros::Publisher sfc_pub;
        ros::Publisher am_points_pub;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud; 

        Eigen::Vector3d goal;
        Eigen::Vector3d last_safe_pos;
        
        pose current_pose;

        std::vector<Eigen::Vector3d> global_setpoints;

        std::vector<Eigen::Vector4d> no_fly_zone;
        std::vector<Eigen::Vector4d> color_range;

        double simulation_hz;
        double map_hz;
        double safety_horizon;
        double reached_threshold;
        double last_safe_yaw;
        double end_time_tol = 0.10;
        double end_dist_tol = 0.10;
        double e_stop_time = 0.25;

        int degree;
        int state;

        bool is_safe = false;
        bool simulation = false;
        bool have_global_cloud = false;
        bool sample_tree = false;
        bool is_enu = false;
        bool init_first_pose = false;
        bool init_cloud = false;
        bool emergency_stop = false;

        t_p_sc emergency_stop_time;

        /** 
         * @brief Callbacks, mainly for loading pcl and commands 
        **/
        void command_callback(const geometry_msgs::PoseStampedConstPtr& msg);
        void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);

        /** 
         * @brief Timers for searching and agent movement 
        **/
        ros::Timer planning_timer; 
        ros::Timer command_timer; 
        ros::Timer map_timer;

        void plasto_timer(const ros::TimerEvent &);
        void agent_command_timer(const ros::TimerEvent &);
        void local_map_timer(const ros::TimerEvent &);

        void get_uav_orientation(
            Eigen::Vector3d acc, double yaw_rad, Eigen::Quaterniond &q, Eigen::Matrix3d &r);

        visualization_msgs::Marker visualize_line_list(
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert, 
            Eigen::Vector4d color, double scale, int index, double transparency);
        
        visualization_msgs::Marker visualize_points(
            vector<Eigen::Vector3d> points_vect, 
            Eigen::Vector4d color, double scale, int index);
        
        nav_msgs::Path vector_3d_to_path(
            vector<Vector3d> path_vector);
        
        visualization_msgs::MarkerArray 
            visualize_corridor(
            std::vector<Corridor> &corridors, 
            Eigen::Vector4d color);

        /** 
         * @brief Convert point cloud from ROS sensor 
         * message to pcl point ptr 
        **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_converter(sensor_msgs::PointCloud2 _pc);

        /** 
         * @brief similar to the ROS euler rpy which does not require the tf library, 
         * hence independent of ROS but yield the same rotation 
        **/
        Eigen::Vector3d euler_rpy(Eigen::Matrix3d R);

        // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
        bool ray_triangle_intersect( 
            const Eigen::Vector3d &orig, 
            const Eigen::Vector3d &end, 
            const Eigen::Vector3d &v0, 
            const Eigen::Vector3d &v1, 
            const Eigen::Vector3d &v2, 
            double &t, Eigen::Vector3d &P);

    public:

        int threads;

        plasto_node(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            /** @brief ROS Params */

            int rpl;
            std::vector<double> search_limit_hfov_list, 
                search_limit_vfov_list, height_list, no_fly_zone_list;

            _nh.param<bool>("ros/simulation", simulation, true);
            _nh.param<bool>("ros/have_global_cloud", have_global_cloud, true);
            _nh.param<int>("ros/threads", threads, -1);
            _nh.param<double>("ros/simulation_hz", simulation_hz, -1.0);
            _nh.param<double>("ros/map_hz", map_hz, -1.0);
            _nh.param<bool>("ros/using_enu", is_enu, false);

            _nh.param<double>("planning/runtime_error", rrt_param.r_e, -1.0);
            _nh.param<double>("planning/refinement_time", rrt_param.r_t, -1.0);
            _nh.param<double>("planning/sensor_range", rrt_param.s_r, -1.0);
            m_p.s_r = rrt_param.s_r;
            _nh.param<double>("planning/sensor_buffer_multiplier", rrt_param.s_bf, -1.0);
            _nh.param<double>("planning/interval", rrt_param.s_i, -1.0);  
            _nh.param<double>("planning/resolution", rrt_param.r, -1.0);
            _nh.getParam("planning/search_limit_hfov", search_limit_hfov_list);
            rrt_param.s_l_h.first = search_limit_hfov_list[0];
            rrt_param.s_l_h.second = search_limit_hfov_list[1];
            _nh.getParam("planning/search_limit_vfov", search_limit_vfov_list);
            rrt_param.s_l_v.first = search_limit_vfov_list[0];
            rrt_param.s_l_v.second = search_limit_vfov_list[1];
            _nh.param<double>("planning/scaled_min_dist_from_node", rrt_param.s_d_n, -1.0);
            _nh.getParam("planning/height", height_list);
            rrt_param.h_c.first = height_list[0];
            rrt_param.h_c.second = height_list[1];
            _nh.getParam("planning/no_fly_zone", no_fly_zone_list);
            if (!no_fly_zone_list.empty())
            {
                for (int i = 0; i < (int)no_fly_zone_list.size() / 4; i++)
                    no_fly_zone.push_back(
                        Eigen::Vector4d(
                        no_fly_zone_list[0+i*4],
                        no_fly_zone_list[1+i*4],
                        no_fly_zone_list[2+i*4],
                        no_fly_zone_list[3+i*4])
                    );
            }
            _nh.param<bool>("planning/sample_tree", sample_tree, false);

            _nh.param<double>("map/resolution", m_p.m_r, -1.0);
            _nh.param<double>("map/size", map_size, -1.0);
            _nh.param<double>("map/vfov", m_p.vfov, -1.0);
            _nh.param<double>("map/hfov", m_p.hfov, -1.0);

            _nh.param<double>("sliding_map/size", m_p.s_m_s, -1.0);

            _nh.param<double>("amtraj/weight/time_regularization", a_m_p.w_t, -1.0);
            _nh.param<double>("amtraj/weight/acceleration", a_m_p.w_a, -1.0);
            _nh.param<double>("amtraj/weight/jerk", a_m_p.w_j, -1.0);
            _nh.param<double>("amtraj/limits/max_vel", a_m_p.m_v, -1.0);
            _nh.param<double>("amtraj/limits/max_acc", a_m_p.m_a, -1.0);
            _nh.param<int>("amtraj/limits/iterations", a_m_p.m_i, -1);
            _nh.param<double>("amtraj/limits/epsilon", a_m_p.e, -1.0);

            _nh.param<double>("safety/total_safety_horizon", safety_horizon, -1.0);
            _nh.param<double>("safety/reached_threshold", reached_threshold, -1.0);

            pcl2_msg_sub = 
                _nh.subscribe<sensor_msgs::PointCloud2>(
                "map", 1,  boost::bind(&plasto_node::pointcloud_callback, this, _1));
            command_sub = 
                _nh.subscribe<geometry_msgs::PoseStamped>(
                "goal", 1,  boost::bind(&plasto_node::command_callback, this, _1));
            pose_sub = 
                _nh.subscribe<geometry_msgs::PoseStamped>(
                "pose", 1,  boost::bind(&plasto_node::pose_callback, this, _1));

            /** @brief For debug */
            local_pcl_pub = 
                _nh.advertise<sensor_msgs::PointCloud2>("local_map", 10);
            pose_pub = 
                _nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
            g_rrt_points_pub = 
                _nh.advertise<nav_msgs::Path>("rrt_points_global", 10);
            am_points_pub = 
                _nh.advertise<nav_msgs::Path>("am_global", 10);
            debug_pcl_pub = 
                _nh.advertise<sensor_msgs::PointCloud2>("debug_map", 10);
            debug_pub = 
                _nh.advertise<visualization_msgs::Marker>("debug_points", 10);
            safe_corridor_pub = 
                _nh.advertise<visualization_msgs::Marker>("safe_corridor", 10);
            command_pub = 
                _nh.advertise<mavros_msgs::PositionTarget>("command", 10);
            sfc_pub = 
                _nh.advertise<visualization_msgs::MarkerArray>("sfc", 1);

            /** @brief Timer for the rrt search and agent */
		    planning_timer = 
                _nh.createTimer(ros::Duration(
                rrt_param.s_i), &plasto_node::plasto_timer, this, false, false);
            command_timer = 
                _nh.createTimer(ros::Duration(
                simulation ? 1/simulation_hz : (rrt_param.s_i / 2)), 
                &plasto_node::agent_command_timer, this, false, false);
            map_timer = 
                _nh.createTimer(ros::Duration(
                1/map_hz), &plasto_node::local_map_timer, this, false, false);

            rrt.set_no_fly_zone(no_fly_zone);

            /** 
             * @brief Choose a color for 
             * visualization elements using random values 
            **/
            std::random_device dev;
            std:mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            int color_count = 5;
            for (int i = 0; i < color_count; i++)
                color_range.push_back(Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5));

            if (simulation)
            {
                // Generate a random point
                std::uniform_real_distribution<double> dis_angle(-M_PI, M_PI);
                std::uniform_real_distribution<double> dis_height(height_list[0], height_list[1]);
                double rand_angle = dis_angle(generator);
                double opp_rand_angle = constrain_between_180(rand_angle - M_PI);

                // std::cout << "rand_angle = " << KBLU << rand_angle << KNRM << " " <<
                //         "opp_rand_angle = " << KBLU << opp_rand_angle << KNRM << std::endl;

                // Start at the random start point
                // Multiply with an safety expansion
                double h = map_size / 2.0 * 1.5; 
                current_pose.pos = Eigen::Vector3d(h * cos(rand_angle), 
                    h * sin(rand_angle), dis_height(generator));
            
                last_safe_pos = current_pose.pos;
                init_first_pose = true;
            }
            if (!have_global_cloud)
            {
                sm.set_parameters(
                    m_p.hfov, m_p.vfov, m_p.m_r, 
                    m_p.s_r, m_p.s_m_s,
                    full_cloud, true);
            }   
            
            local_cloud = 
                pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
            
            if (!have_global_cloud)
                sm.set_parameters(
                    m_p.hfov, m_p.vfov, m_p.m_r, 
                    m_p.s_r, m_p.s_m_s,
                    full_cloud, true);
            else
                map_timer.start();

            rrt.set_parameters(rrt_param);

            state = agent_state::IDLE;
                
            command_timer.start();
            planning_timer.start();
        }

        ~plasto_node()
        {
            // Clear all the points within the clouds
            full_cloud->points.clear();
            local_cloud->points.clear();

            // Stop all the timers
            command_timer.stop();
            planning_timer.stop();
            map_timer.stop();
        }

        void get_pose_goal_data(
            Eigen::Vector3d &goal_copy,
            pose &pose_copy)
        {
            std::lock(pose_update_mutex, goal_update_mutex);

            pose_copy = current_pose;
            goal_copy = goal;

            pose_update_mutex.unlock();
            goal_update_mutex.unlock();
        }

        void publish_or_update_command(
            Eigen::Vector3d pos, Eigen::Vector3d vel, 
            Eigen::Vector3d acc, double yaw)
        {
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

                get_uav_orientation(
                    acc, current_pose.rot.e.z(), 
                    current_pose.rot.q, current_pose.rot.r);

                pose.pose.orientation.w = current_pose.rot.q.w();
                pose.pose.orientation.x = current_pose.rot.q.x();
                pose.pose.orientation.y = current_pose.rot.q.y();
                pose.pose.orientation.z = current_pose.rot.q.z();

                pose_pub.publish(pose);

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

                command_pub.publish(target);
            }
        }

        std::vector<Eigen::Vector3d> get_am_path_interval(double interval)
        {
            std::vector<Eigen::Vector3d> am_path;
            for (int i = 0; i < (int)am.size(); i++)
            {
                double total_duration = 
                    duration<double>(am[i].s_e_t.second - 
                    am[i].s_e_t.first).count();
                
                double split = ceil(total_duration / interval); 
                double corrected_interval = total_duration / split;
                double accumulated = 0.0;
                for (int k = 0; k < (int)split; k++)
                {
                    if (accumulated > total_duration)
                        continue;
                    am_path.push_back(
                        am[i].traj.getPos(accumulated));
                    accumulated += corrected_interval;
                }
            }

            return am_path;
        }

};

#endif