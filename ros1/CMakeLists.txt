cmake_minimum_required(VERSION 3.0.2)
project(plasto)

# Set target C++ standard and required compiler features
# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

## Find catkin macros and libraries
find_package(catkin 
  COMPONENTS
    roscpp
    common_msgs
    mavros_msgs
    pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  CATKIN_DEPENDS  
    roscpp 
    common_msgs
    mavros_msgs
    pcl_ros

  DEPENDS
    Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

# how to add external libraries not as subdirectory
# https://github.com/SRombauts/SQLiteCpp/issues/250#issuecomment-569876565
add_subdirectory(../lib_lro_rrt lro_rrt)
add_subdirectory(../lib_octree_local_map lib_olm)

add_executable(${PROJECT_NAME}_node 
    src/corridor_gen.cpp
    src/callbacks.cpp
    src/main.cpp
    src/plasto.cpp
    src/helper.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
    ${PCL_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  lro_rrt
  lib_olm
)