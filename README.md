# Limited Range Octree RRT on ROS

## Introduction
lro_rrt_ros serves as wrapper around the lib_lro_rrt server (`lib_lro_rrt` package) where the input is a **pointcloud** (using the PCL library), **current pose** and **destination** and finds a collision free path in sub millisecond.

Some benefits in this module
- Using `lib_lro_rrt` which searches for obstacle free path with 2 passes (Runs a modified version of RRT, and next is to trim/shorten the path)
- This search adopts the same mindset as https://github.com/mit-acl/faster where the search will always return with a solution, as the search takes into account the local environment
- Using `mockamap` from `HKUST` https://github.com/HKUST-Aerial-Robotics/mockamap
- Using a "LIDAR" kind of sensor, that returns the surface of the terrain, this utilizes the modified octree function from `lib_lro_rrt`. `raycast time (10.8249ms)` with 640 lines
- Using `libnbspline` which provides a smooth trajectory

| preview | random_fov |
| :--: | :--: |
| [<img src="lro_rrt.gif" width="500"/>](lro_rrt.gif) | [<img src="lro_rrt_range.jpg" width="450"/>](lro_rrt_range.jpg) |

## Setup
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/matthewoots/lro_rrt_ros --recurse-submodules
cd ..
catkin build
```

## How to Run this module
Open 2 terminals

1. On the first terminal run `roslaunch lro_rrt_ros sample.launch` which will launch an rviz display of the map and the pose of the agent.

2. On the second terminal run `python3 send_command_auto.py` in the `<lro_rrt_ros>/ros1` directory. This runs a script to generate a random point that is on the opposite side of the map and pass that `goal` to the search node.  
```bash
$ python3 send_command_auto.py
[1] publish new point
[2] publish new point
[3] publish new point
[4] publish new point
...
```