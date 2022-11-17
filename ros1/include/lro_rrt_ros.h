/*
* lro_rrt_ros.h
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
#ifndef LRO_RRT_ROS_H
#define LRO_RRT_ROS_H

#include "lro_rrt_server.h"
#include "mapper.h"
#include "am_traj.hpp"

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

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace std::chrono; // nanoseconds, system_clock, seconds
using namespace lro_rrt_server;
using namespace octree_map;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

class lro_rrt_ros_node
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
            SWITCH_MISSION,
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

        ros::NodeHandle _nh;

        ros::Subscriber pcl2_msg_sub, command_sub;
        ros::Publisher local_pcl_pub, g_rrt_points_pub;
        ros::Publisher pose_pub, debug_pcl_pub, debug_pub;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, local_cloud; 

        Eigen::Vector3d goal;
        pose current_pose;

        vector<Eigen::Vector4d> no_fly_zone;

        std::vector<Eigen::Vector4d> color_range;

        double simulation_hz, map_hz, duration_committed, default_knot_spacing;
        int degree, state;
        bool is_safe = false, 
            simulation = false, 
            have_global_cloud = false,
            sample_tree = false;
        orientation orientation;

        double safety_horizon, reached_threshold;

        bool init_cloud = false, emergency_stop = false;

        t_p_sc emergency_stop_time;

        /** @brief Callbacks, mainly for loading pcl and commands **/
        void command_callback(const geometry_msgs::PointConstPtr& msg);
        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

        /** @brief Timers for searching and agent movement **/
        ros::Timer search_timer, agent_timer, map_timer;
        void rrt_search_timer(const ros::TimerEvent &);
        void agent_forward_timer(const ros::TimerEvent &);
        void local_map_timer(const ros::TimerEvent &);

        void calc_uav_orientation(
            Eigen::Vector3d acc, double yaw_rad, Eigen::Quaterniond &q, Eigen::Matrix3d &r);

        int error_counter;

        nav_msgs::Path vector_3d_to_path(vector<Vector3d> path_vector)
        {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "world";
            for (int i = 0; i < path_vector.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = path_vector[i][0];
                pose.pose.position.y = path_vector[i][1];
                pose.pose.position.z = path_vector[i][2];
                path.poses.push_back(pose);
            }

            return path;
        }

        visualization_msgs::Marker visualize_points(
            vector<Eigen::Vector3d> points_vect, 
            Eigen::Vector4d color, double scale, int index)
        {
            visualization_msgs::Marker points;
            points.header.frame_id = "world";
            points.header.stamp = ros::Time::now();
            points.type = visualization_msgs::Marker::POINTS;
            points.action = visualization_msgs::Marker::ADD;

            points.id = index;

            points.color.r = color(0);
            points.color.g = color(1);
            points.color.b = color(2);

            points.color.a = color(3);

            points.scale.x = scale;
            points.scale.y = scale;
            
            // Create the vertices point
            for (Eigen::Vector3d &vect : points_vect)
            {
                geometry_msgs::Point p1;
                p1.x = vect.x();
                p1.y = vect.y();
                p1.z = vect.z();

                points.points.push_back(p1);
            }

            return points;
        }

        visualization_msgs::Marker visualize_line_list(
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert, 
            Eigen::Vector4d color, double scale, int index, double transparency)
        {
            visualization_msgs::Marker lines;
            lines.header.frame_id = "world";
            lines.header.stamp = ros::Time::now();
            lines.type = visualization_msgs::Marker::LINE_LIST;
            lines.action = visualization_msgs::Marker::ADD;

            lines.id = index;

            lines.color.r = color(0);
            lines.color.g = color(1);
            lines.color.b = color(2);

            lines.color.a = transparency;

            lines.scale.x = scale;
            
            // Create the vertices line list
            for (std::pair<Eigen::Vector3d, Eigen::Vector3d> &vert_pair : vect_vert)
            {
                geometry_msgs::Point p1, p2;
                p1.x = vert_pair.first.x();
                p2.x = vert_pair.second.x();

                p1.y = vert_pair.first.y();
                p2.y = vert_pair.second.y();

                p1.z = vert_pair.first.z();
                p2.z = vert_pair.second.z();

                lines.points.push_back(p1);
                lines.points.push_back(p2);
            }

            return lines;
        }

    public:

        int threads;

        lro_rrt_ros_node(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
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

            pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
                "/mock_map", 1,  boost::bind(&lro_rrt_ros_node::pcl2_callback, this, _1));
            command_sub = _nh.subscribe<geometry_msgs::Point>(
                "/goal", 1,  boost::bind(&lro_rrt_ros_node::command_callback, this, _1));

            /** @brief For debug */
            local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
            pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
            g_rrt_points_pub = _nh.advertise<nav_msgs::Path>("/rrt_points_global", 10);
            debug_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/debug_map", 10);
            debug_pub = _nh.advertise
                <visualization_msgs::Marker>("/debug_points", 10);

            /** @brief Timer for the rrt search and agent */
		    search_timer = _nh.createTimer(
                ros::Duration(rrt_param.s_i), 
                &lro_rrt_ros_node::rrt_search_timer, this, false, false);
            agent_timer = _nh.createTimer(
                ros::Duration(1/simulation_hz), 
                &lro_rrt_ros_node::agent_forward_timer, this, false, false);
            map_timer = _nh.createTimer(
                ros::Duration(1/map_hz), 
                &lro_rrt_ros_node::local_map_timer, this, false, false);

            /** @brief Choose a color for the trajectory using random values **/
            std::random_device dev;
            std:mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            int color_count = 5;
            for (int i = 0; i < color_count; i++)
                color_range.push_back(Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5));

            if (simulation)
            {
                /** @brief Generate a random point **/
                std::uniform_real_distribution<double> dis_angle(-M_PI, M_PI);
                std::uniform_real_distribution<double> dis_height(height_list[0], height_list[1]);
                double rand_angle = dis_angle(generator);
                double opp_rand_angle = constrain_between_180(rand_angle - M_PI);

                std::cout << "rand_angle = " << KBLU << rand_angle << KNRM << " " <<
                        "opp_rand_angle = " << KBLU << opp_rand_angle << KNRM << std::endl;

                // Let us start at the random start point
                double h = map_size / 2.0 * 1.5; // multiply with an expansion
                current_pose.pos = Eigen::Vector3d(h * cos(rand_angle), 
                    h * sin(rand_angle), dis_height(generator));
            }
            
            local_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
            
            if (!have_global_cloud)
                sm.set_parameters(
                    m_p.hfov, m_p.vfov, m_p.m_r, 
                    m_p.s_r, m_p.s_m_s,
                    full_cloud, true);

            state = agent_state::IDLE;

            if (simulation)
            {
                agent_timer.start();
                map_timer.start();
            }
            
            search_timer.start();
        }

        ~lro_rrt_ros_node()
        {
            // Clear all the points within the clouds
            full_cloud->points.clear();
            local_cloud->points.clear();

            // Stop all the timers
            agent_timer.stop();
            search_timer.stop();
            map_timer.stop();
        }

        /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_converter(sensor_msgs::PointCloud2 _pc)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(_pc, pcl_pc2);

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
            return tmp_cloud;
        }

};

#endif