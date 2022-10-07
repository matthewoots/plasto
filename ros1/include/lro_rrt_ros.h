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
using namespace lro_rrt_server;

class lro_rrt_ros_node
{
    private:

        lro_rrt_server::lro_rrt_server_node rrt;
        lro_rrt_server::lro_rrt_server_node::parameters rrt_param;

        std::mutex pose_update_mutex;

        ros::NodeHandle _nh;

        ros::Subscriber pcl2_msg_sub, command_sub;
        ros::Publisher local_pcl_pub, g_rrt_points_pub;
        ros::Publisher pose_pub, debug_pcl_pub, debug_position_pub;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr _full_cloud, _local_cloud;

        Eigen::Vector3d current_point;

        vector<Eigen::Vector3d> global_search_path;
        vector<Eigen::Vector4d> no_fly_zone;

        Eigen::Vector4d color;

        double simulation_step;
        double agent_step;
        double max_velocity;

        bool received_command = false;

        /** @brief Callbacks, mainly for loading pcl and commands **/
        void command_callback(const geometry_msgs::PointConstPtr& msg);
        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    
        /** @brief Functions used in lro_rrt **/
        bool check_and_update_search(Eigen::Vector3d first_cp);
        void generate_search_path();

        /** @brief Timers for searching and agent movement **/
        ros::Timer search_timer, agent_timer;
        void rrt_search_timer(const ros::TimerEvent &);
        void agent_forward_timer(const ros::TimerEvent &);

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

        void visualize_points(double scale_small, double scale_big)
        {
            visualization_msgs::Marker sphere_points, search;
            sphere_points.header.frame_id = search.header.frame_id = "world";
            sphere_points.header.stamp = search.header.stamp = ros::Time::now();
            sphere_points.type = visualization_msgs::Marker::SPHERE;
            search.type = visualization_msgs::Marker::SPHERE;
            sphere_points.action = search.action = visualization_msgs::Marker::ADD;

            sphere_points.id = 0;
            search.id = 1;

            sphere_points.pose.orientation.w = search.pose.orientation.w = 1.0;
            sphere_points.color.r = search.color.g = color(0);
            sphere_points.color.g = search.color.r = color(1);
            sphere_points.color.b = search.color.b = color(2);

            sphere_points.color.a = color(3);
            search.color.a = 0.1;

            sphere_points.scale.x = scale_small;
            sphere_points.scale.y = scale_small;
            sphere_points.scale.z = scale_small;

            search.scale.x = scale_big;
            search.scale.y = scale_big;
            search.scale.z = scale_big;

        }

    public:

        lro_rrt_ros_node(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            /** @brief ROS Params */
            _nh.param<double>("sub_runtime_error", rrt_param.r_e.first, -1.0);
            _nh.param<double>("runtime_error", rrt_param.r_e.second, -1.0);
            _nh.param<double>("simulation_step", simulation_step, -1.0);

            _nh.param<double>("sensor_range", rrt_param.s_r, -1.0);
            _nh.param<double>("sensor_buffer_multiplier", rrt_param.s_bf, -1.0);
            _nh.param<double>("protected_zone", rrt_param.p_z, -1.0);

            _nh.param<double>("search_interval", rrt_param.s_i, -1.0);  
            _nh.param<double>("resolution", rrt_param.r, -1.0); 

            _nh.param<double>("map_size", rrt_param.m_s, -1.0);
            _nh.param<double>("max_velocity", max_velocity, -1.0);

            std::vector<double> height_list;
            _nh.getParam("height", height_list);
            rrt_param.h_c.first = height_list[0];
            rrt_param.h_c.second = height_list[1];

            std::vector<double> no_fly_zone_list;
            _nh.getParam("no_fly_zone", no_fly_zone_list);
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

            pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
                "/mock_map", 1,  boost::bind(&lro_rrt_ros_node::pcl2_callback, this, _1));
            command_sub = _nh.subscribe<geometry_msgs::Point>(
                "/goal", 1,  boost::bind(&lro_rrt_ros_node::command_callback, this, _1));

            /** @brief For debug */
            local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
            pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
            g_rrt_points_pub = _nh.advertise<nav_msgs::Path>("/rrt_points_global", 10);
            debug_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/debug_map", 10);
            debug_position_pub = _nh.advertise
                <visualization_msgs::Marker>("/debug_points", 10);

            /** @brief Timer for the rrt search and agent */
		    search_timer = _nh.createTimer(
                ros::Duration(rrt_param.s_i), 
                &lro_rrt_ros_node::rrt_search_timer, this, false, false);
            agent_timer = _nh.createTimer(
                ros::Duration(0.01), 
                &lro_rrt_ros_node::agent_forward_timer, this, false, false);

            /** @brief Choose a color for the trajectory using random values **/
            std::random_device dev;
            std:mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            color = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

            /** @brief Generate a random point **/
            std::uniform_real_distribution<double> dis_angle(-M_PI, M_PI);
            std::uniform_real_distribution<double> dis_height(height_list[0], height_list[1]);
            double rand_angle = dis_angle(generator);
            double opp_rand_angle = constrain_to_pi(rand_angle - M_PI);

            std::cout << "rand_angle = " << KBLU << rand_angle << KNRM << " " <<
                    "opp_rand_angle = " << KBLU << opp_rand_angle << KNRM << std::endl;

            double h = rrt_param.m_s / 2.0 * 1.5; // multiply with an expansion
            Eigen::Vector3d start = Eigen::Vector3d(h * cos(rand_angle), 
                h * sin(rand_angle), dis_height(generator));
            // Eigen::Vector3d end = Eigen::Vector3d(h * cos(opp_rand_angle), 
            //     h * sin(opp_rand_angle), dis_height(generator));

            // std::cout << "start_position = " << KBLU << start.transpose() << KNRM << " " <<
            //         "end_position = " << KBLU << end.transpose() << KNRM << " " <<
            //         "distance = " << KBLU << (start-end).norm() << KNRM << std::endl;

            // Let us start at the random start point
            current_point = start;
            agent_step = max_velocity * simulation_step;

            agent_timer.start();
            search_timer.start();
        }

        double constrain_to_pi(double x){
            x = fmod(x + M_PI, 2 * M_PI);
            if (x < 0)
                x += 2 * M_PI;
            return x - M_PI;
        }


        ~lro_rrt_ros_node()
        {
            // Clear all the points within the clouds
            _full_cloud->points.clear();
            _local_cloud->points.clear();

            // Stop all the timers
            agent_timer.stop();
            search_timer.stop();
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