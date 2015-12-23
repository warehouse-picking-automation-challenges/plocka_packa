cmake_minimum_required(VERSION 2.8.3)
project(apc_json)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  apc_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

