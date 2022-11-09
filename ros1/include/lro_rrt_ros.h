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
#include "nbspline.h"

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
using namespace nbspline;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

class lro_rrt_ros_node
{
    private:

        struct map_parameters
        {
            double h_d; // horizontal fov length
            double v_d; // vertical fov length
            int h_p; // horizontal pixel
            int v_p; // vertical pixel
            double h_s; // angle step for horizontal
            double v_s; // angle step for vertical
            double m_r; // map resolution
            uint r_p_l; // ray per layer
            double vfov;
            double hfov;
            double s_m_s; // sliding map size
        };

        struct bspline_parameters
        {
            vector<Eigen::Vector3d> c_p_v; // control point vector
            vector<t_p_sc> t_p_v; // time point vector
        };

        struct orientation
        {
            Eigen::Vector3d e; // euler angles
            Eigen::Quaterniond q; // quaternion
            Eigen::Matrix3d r; // rotation matrix
        };

        lro_rrt_server::lro_rrt_server_node rrt, map, sliding_map;
        lro_rrt_server::lro_rrt_server_node::parameters rrt_param;
        map_parameters m_p;
        vector<Eigen::Vector3d> sensing_offset;

        bspline_parameters b_p; // current bspline parameters
        nbspline::bspline_trajectory b_t;

        std::mutex pose_update_mutex;

        ros::NodeHandle _nh;

        ros::Subscriber pcl2_msg_sub, command_sub;
        ros::Publisher local_pcl_pub, g_rrt_points_pub;
        ros::Publisher pose_pub, debug_pcl_pub, debug_position_pub;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, local_cloud; 
        // pcl::PointCloud<pcl::PointXYZ>::Ptr sliding_cloud;

        Eigen::Vector3d current_point, goal;

        vector<Eigen::Vector3d> global_search_path;
        vector<Eigen::Vector4d> no_fly_zone;

        Eigen::Vector4d color;

        double simulation_hz, map_hz;
        // double agent_step;
        double max_velocity;
        double default_knot_spacing;
        int degree;
        double duration_committed;
        bool valid;
        orientation orientation;

        t_p_sc traj_start_time;

        bool received_command = false;
        bool init_cloud = false;

        /** @brief Callbacks, mainly for loading pcl and commands **/
        void command_callback(const geometry_msgs::PointConstPtr& msg);
        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    
        /** @brief Functions used in lro_rrt **/
        bool check_and_update_search();
        bool generate_search_path();

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

        int threads;

        lro_rrt_ros_node(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            /** @brief ROS Params */

            int rpl;
            std::vector<double> search_limit_hfov_list, 
                search_limit_vfov_list, height_list, no_fly_zone_list;

            _nh.param<int>("ros/threads", threads, -1);
            _nh.param<double>("ros/simulation_hz", simulation_hz, -1.0);
            _nh.param<double>("ros/map_hz", map_hz, -1.0);

            _nh.param<double>("planning/sub_runtime_error", rrt_param.r_e.first, -1.0);
            _nh.param<double>("planning/runtime_error", rrt_param.r_e.second, -1.0);
            _nh.param<double>("planning/sensor_range", rrt_param.s_r, -1.0);
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

            _nh.param<double>("map/resolution", m_p.m_r, -1.0);
            _nh.param<int>("map/hpixel", m_p.h_p, -1);
            _nh.param<int>("map/vpixel", m_p.v_p, -1);
            _nh.param<double>("map/size", rrt_param.m_s, -1.0);
            _nh.param<double>("map/max_velocity", max_velocity, -1.0);
            _nh.param<double>("map/vfov", m_p.vfov, -1.0);
            _nh.param<double>("map/hfov", m_p.hfov, -1.0);
            _nh.param<double>("map/sliding_map_size", m_p.s_m_s, -1.0);

            _nh.param<double>("bspline/default_knot_spacing", default_knot_spacing, -1.0);
            _nh.param<int>("bspline/degree", degree, -1);
            _nh.param<double>("bspline/duration_committed", duration_committed, -1.0);

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
                ros::Duration(1/simulation_hz), 
                &lro_rrt_ros_node::agent_forward_timer, this, false, false);
            map_timer = _nh.createTimer(
                ros::Duration(1/map_hz), 
                &lro_rrt_ros_node::local_map_timer, this, false, false);

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
            // agent_step = max_velocity * 1/simulation_hz;

            m_p.v_d = 2.0 * rrt_param.s_r * tan(m_p.vfov/2.0);
            m_p.h_d = 2.0 * rrt_param.s_r * sin(m_p.hfov/2.0);

            m_p.v_s = m_p.vfov / (double)m_p.v_p;
            m_p.h_s = m_p.hfov / (double)m_p.h_p;

            for (int i = 0; i < m_p.v_p; i++)
                for (int j = 0; j < m_p.h_p; j++)
                {
                    Eigen::Vector3d q = Eigen::Vector3d(
                        rrt_param.s_r * cos(j*m_p.h_s - m_p.hfov/2.0),
                        rrt_param.s_r * sin(j*m_p.h_s - m_p.hfov/2.0),
                        rrt_param.s_r * tan(i*m_p.v_s - m_p.vfov/2.0)
                    );
                    sensing_offset.push_back(q);
                }
            
            local_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
            // sliding_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            //     new pcl::PointCloud<pcl::PointXYZ>());

            agent_timer.start();
            search_timer.start();
            map_timer.start();
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
        
         
        pcl::PointCloud<pcl::PointXYZ>::Ptr raycast_pcl_w_fov(Eigen::Vector3d p)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);

            for (int i = 0; i < (int)sensing_offset.size(); i++)
            {
                Eigen::Quaterniond point;
                point.w() = 0;
                point.vec() = sensing_offset[i];
                Eigen::Quaterniond rotatedP = orientation.q * point * orientation.q.inverse(); 
                Eigen::Vector3d q = p + rotatedP.vec();

                Eigen::Vector3d intersect;
                // Eigen::Vector3d q = p + sensing_offset[i];
                if (!map.check_approx_intersection_by_segment(
                    p, q, (float)(m_p.m_r), intersect))
                {
                    Eigen::Vector3d direction = (q - p).normalized(); 
                    intersect -= rrt_param.r/2 * direction;
                    pcl::PointXYZ add;
                    add.x = intersect.x();
                    add.y = intersect.y();
                    add.z = intersect.z();
                    tmp->points.push_back(add);
                }
            }

            return tmp;

        }
};

#endif