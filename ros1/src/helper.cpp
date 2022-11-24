/*
* helper.cpp
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

nav_msgs::Path plasto_node::vector_3d_to_path(
    vector<Vector3d> path_vector)
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

visualization_msgs::Marker 
    plasto_node::visualize_points(
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

visualization_msgs::MarkerArray 
    plasto_node::visualize_corridor(
    std::vector<Corridor> &corridors, 
    Eigen::Vector4d color)
{
    visualization_msgs::MarkerArray corridor_array;
    int index = 0;
    for (auto corridor : corridors)
    {
        auto [position, radius] = corridor;
        visualization_msgs::Marker corridor_sphere;
        corridor_sphere.header.frame_id = "world";
        corridor_sphere.header.stamp = ros::Time::now();
        corridor_sphere.id = index;
        corridor_sphere.type = visualization_msgs::Marker::SPHERE;
        corridor_sphere.action = visualization_msgs::Marker::ADD;
        corridor_sphere.pose.position.x = position.x();
        corridor_sphere.pose.position.y = position.y();
        corridor_sphere.pose.position.z = position.z();
        corridor_sphere.pose.orientation.x = 0.0;
        corridor_sphere.pose.orientation.y = 0.0;
        corridor_sphere.pose.orientation.z = 0.0;
        corridor_sphere.pose.orientation.w = 1.0;
        corridor_sphere.scale.x = radius * 2;
        corridor_sphere.scale.y = radius * 2;
        corridor_sphere.scale.z = radius * 2;
        corridor_sphere.color.a = 0.4; // Don't forget to set the alpha!
        corridor_sphere.color.r = color[0];
        corridor_sphere.color.g = color[1];
        corridor_sphere.color.b = color[2];

        corridor_array.markers.emplace_back(corridor_sphere);

        index++;
    }

    return corridor_array;
}

visualization_msgs::Marker 
    plasto_node::visualize_line_list(
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

/**
 * @brief visualize_triangle_list
 * https://github.com/Dung-Han-Lee/Convexhull-3D-Implementation-of-incremental-convexhull-algorithm/blob/master/src/demo.cpp
 * @param tri 
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker 
    plasto_node::visualize_triangle_list(Eigen::Vector4d color,
    std::vector<octree_map::sliding_map::triangles> tri)
{
    visualization_msgs::Marker tri_marker;
    tri_marker.action = visualization_msgs::Marker::ADD;
    tri_marker.scale.x = tri_marker.scale.y = tri_marker.scale.z = 1.0;
    tri_marker.pose.orientation.x = 0.0;
    tri_marker.pose.orientation.y = 0.0;
    tri_marker.pose.orientation.z = 0.0;
    tri_marker.pose.orientation.w = 1.0;
    tri_marker.pose.position.x = 0.0;
    tri_marker.pose.position.y = 0.0;
    tri_marker.pose.position.z = 0.0;
    tri_marker.header.frame_id = "world";
    tri_marker.header.stamp = ros::Time::now();
    tri_marker.id = 0;
    tri_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    tri_marker.color.r = color(0);
    tri_marker.color.g = color(1);
    tri_marker.color.b = color(2);

    tri_marker.color.a = 0.05;

    geometry_msgs::Point temp;
    for (const auto& triangles : tri)
        for (const auto& triangle_index : triangles.tri_idx) 
        {
            for(int i = 0; i < 3; i++)
            {
                temp.x = triangles.vert[triangle_index[i]].x(); 
                temp.y = triangles.vert[triangle_index[i]].y(); 
                temp.z = triangles.vert[triangle_index[i]].z(); 
                tri_marker.points.push_back(temp);
            }
        }
    
    return tri_marker;
}

/** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
pcl::PointCloud<pcl::PointXYZ>::Ptr 
    plasto_node::pcl2_converter(sensor_msgs::PointCloud2 _pc)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(_pc, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
    
    return tmp_cloud;
}

/** 
 * @brief similar to the ROS euler rpy which does not require the tf library, 
 * hence independent of ROS but yield the same rotation 
**/
Eigen::Vector3d plasto_node::euler_rpy(
    Eigen::Matrix3d R)
{
    Eigen::Vector3d euler_out;
    // Each vector is a row of the matrix
    Eigen::Vector3d m_el[3];
    m_el[0] = Vector3d(R(0,0), R(0,1), R(0,2));
    m_el[1] = Vector3d(R(1,0), R(1,1), R(1,2));
    m_el[2] = Vector3d(R(2,0), R(2,1), R(2,2));

    // Check that pitch is not at a singularity
    if (abs(m_el[2].x()) >= 1)
    {
        euler_out.z() = 0;

        // From difference of angles formula
        double delta = atan2(m_el[2].y(),m_el[2].z());
        if (m_el[2].x() < 0)  //gimbal locked down
        {
            euler_out.y() = M_PI / 2.0;
            euler_out.x() = delta;
        }
        else // gimbal locked up
        {
            euler_out.y() = -M_PI / 2.0;
            euler_out.x() = delta;
        }
    }
    else
    {
        euler_out.y() = - asin(m_el[2].x());

        euler_out.x() = atan2(m_el[2].y()/cos(euler_out.y()), 
            m_el[2].z()/cos(euler_out.y()));

        euler_out.z() = atan2(m_el[1].x()/cos(euler_out.y()), 
            m_el[0].x()/cos(euler_out.y()));
    }

    return euler_out;
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
bool plasto_node::ray_triangle_intersect( 
    const Eigen::Vector3d &orig, 
    const Eigen::Vector3d &end, 
    const Eigen::Vector3d &v0, 
    const Eigen::Vector3d &v1, 
    const Eigen::Vector3d &v2, 
    double &t, Eigen::Vector3d &P) 
{ 
    Eigen::Vector3d dir = (end - orig).normalized();
    double dist = (end - orig).norm();

    double kEpsilon = 0.0001;
    // compute plane's normal
    Eigen::Vector3d v0v1 = v1 - v0; 
    Eigen::Vector3d v0v2 = v2 - v0; 
    // no need to normalize
    Eigen::Vector3d N = v0v1.cross(v0v2);  //N 
    double area2 = N.norm(); 

    // Step 1: finding P

    // check if ray and plane are parallel.
    double NdotRayDirection = 
        N.x() * dir.x() + 
        N.y() * dir.y() +
        N.z() * dir.z(); 
    if (abs(NdotRayDirection) < kEpsilon)  //almost 0 
        return false;  //they are parallel so they don't intersect ! 

    // compute d parameter using equation 2
    float d = 
        -(N.x() * v0.x() + 
        N.y() * v0.y() +
        N.z() * v0.z()); 

    // compute t (equation 3)
    t = -((N.x() * orig.x() + 
        N.y() * orig.y() +
        N.z() * orig.z()) + d) / NdotRayDirection; 

    // check if the triangle is in behind the ray
    if (t < 0) return false;  //the triangle is behind 
    if (t > dist) return false;  //the triangle is infront of the line 

    // compute the intersection point using equation 1
    P = orig + t * dir; 

    // Step 2: inside-outside test
    Eigen::Vector3d C;  //vector perpendicular to triangle's plane 

    // edge 0
    Eigen::Vector3d edge0 = v1 - v0; 
    Eigen::Vector3d vp0 = P - v0; 
    C = edge0.cross(vp0); 
    if ((N.x() * C.x() + 
        N.y() * C.y() +
        N.z() * C.z()) < 0) return false;  //P is on the right side 

    // edge 1
    Eigen::Vector3d edge1 = v2 - v1; 
    Eigen::Vector3d vp1 = P - v1; 
    C = edge1.cross(vp1); 
    if ((N.x() * C.x() + 
        N.y() * C.y() +
        N.z() * C.z()) < 0)  return false;  //P is on the right side 

    // edge 2
    Eigen::Vector3d edge2 = v0 - v2; 
    Eigen::Vector3d vp2 = P - v2; 
    C = edge2.cross(vp2); 
    if ((N.x() * C.x() + 
        N.y() * C.y() +
        N.z() * C.z()) < 0) return false;  //P is on the right side; 

    return true;  //this ray hits the triangle 
}

void plasto_node::get_uav_orientation(
	Eigen::Vector3d acc, double yaw_rad, 
    Eigen::Quaterniond &q, Eigen::Matrix3d &r)
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