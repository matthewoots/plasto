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