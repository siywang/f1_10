cmake_minimum_required(VERSION 2.8.3)
project(siyanwang_reactive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(siyanwang_reactive src/siyanwang_reactive_gap_follow_node.cpp)


 target_link_libraries(siyanwang_reactive
   ${catkin_LIBRARIES}
 )


