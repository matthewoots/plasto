/*
* tbborrt_ros.h
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
#ifndef TBBORRT_ROS_H
#define TBBORRT_ROS_H

#include "tbborrt_server.h"

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
using namespace tbborrt_server;

class tbborrt_ros_node
{
    private:

        tbborrt_server::tbborrt_server_node rrt;

        std::mutex search_points_mutex, pose_update_mutex, cloud_mutex;

        ros::NodeHandle _nh;
        ros::Subscriber pcl2_msg_sub;
        ros::Publisher local_pcl_pub, g_rrt_points_pub;
        ros::Publisher pose_pub, debug_pcl_pub, debug_position_pub;
        
        std::pair<double,double> _runtime_error, _height_constrain;

        double _search_interval;
        double _sensor_range, _obstacle_threshold;
        double _resolution;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _full_cloud, _local_cloud;

        Eigen::Vector3d start, end, current_point;

        vector<Eigen::Vector3d> previous_search_points;
        vector<Eigen::Vector3d> global_search_path;

        vector<Eigen::Vector4d> _no_fly_zone;

        Eigen::Vector4d color;

        ros::Time last_pcl_msg;

        ros::Timer search_timer;

        std::pair<Eigen::Vector3d, Eigen::Vector3d> start_end;

        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    
        bool check_and_update_search(Eigen::Vector3d first_cp);

        void generate_search_path();

        void run_search_timer(const ros::TimerEvent &);

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
            
            // geometry_msgs::Point pt;
            // pt.x = direct_goal[0];
            // pt.y = direct_goal[1];
            // pt.z = direct_goal[2];

            // geometry_msgs::Point ctr;
            // ctr.x = current_point[0];
            // ctr.y = current_point[1];
            // ctr.z = current_point[2];

            // sphere_points.pose.position = pt;
            // search.pose.position = ctr;

            // debug_position_pub.publish(sphere_points);
            // debug_position_pub.publish(search);

        }

    public:

        tbborrt_ros_node(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            /** @brief ROS Params */
            _nh.param<double>("sub_runtime_error", _runtime_error.first, 0.02);
            _nh.param<double>("runtime_error", _runtime_error.second, 0.1);

            _nh.param<double>("sensor_range", _sensor_range, 5.0);
            _nh.param<double>("threshold", _obstacle_threshold, 0.3);

            _nh.param<double>("search_interval", _search_interval, 0.5);  
            _nh.param<double>("resolution", _resolution, 0.25); 

            std::vector<double> height_list;
            _nh.getParam("height", height_list);
 
            _height_constrain.first = height_list[0];
            _height_constrain.second = height_list[1];

            pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
                "/mock_map", 1,  boost::bind(&tbborrt_ros_node::pcl2_callback, this, _1));

            /** @brief For debug */
            local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
            pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);
            g_rrt_points_pub = _nh.advertise<nav_msgs::Path>("/rrt_points_global", 10);
            debug_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/debug_map", 10);
            debug_position_pub = _nh.advertise
                <visualization_msgs::Marker>("/debug_points", 10);

            /** @brief Timer that when to search */
		    search_timer = _nh.createTimer(
                ros::Duration(_search_interval), 
                &tbborrt_ros_node::run_search_timer, this, false, false);

            // Choose a color for the trajectory using random values
            std::random_device dev;
            std:mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            color = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);


            // current_point = start;

            search_timer.start();
        }


        ~tbborrt_ros_node()
        {
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