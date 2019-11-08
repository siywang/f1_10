cmake_minimum_required(VERSION 2.8.3)
project(saz_pure_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  visualization_msgs
  sensor_msgs
)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(saz_pure_pursuit src/pure_pursuit.cpp)


target_link_libraries(saz_pure_pursuit
  ${catkin_LIBRARIES}
)

