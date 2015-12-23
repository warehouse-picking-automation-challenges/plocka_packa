cmake_minimum_required(VERSION 2.8.3)
project(apc_simulator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  #gazebo_msgs
  geometry_msgs
)

#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc_simulator
#  CATKIN_DEPENDS roscpp rospy std_msgs tf gazebo_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(        kinect_tf_publisher src/kinect_tf_publisher.cpp)
#target_link_libraries( kinect_tf_publisher ${catkin_LIBRARIES})