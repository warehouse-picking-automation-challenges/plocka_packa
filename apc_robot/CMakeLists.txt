cmake_minimum_required(VERSION 2.8.3)
project(apc_robot)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  sensor_msgs
  control_msgs
  actionlib
  tf
  visualization_msgs
  apc_msgs
  move_base_msgs
  #robot_mechanism_controllers
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apc_robot_moveit apc_robot_grippers pr2_head pr2_torso apc_arms_cartesian pr2_base apc_arms_joint
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rospy std_msgs sensor_msgs control_msgs actionlib tf visualization_msgs apc_msgs move_base_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Executables
add_executable(         pr2_base_node       src/base.cpp)
target_link_libraries(  pr2_base_node   	${catkin_LIBRARIES})
add_dependencies(       pr2_base_node       move_base_msgs_generate_messages_cpp)

add_executable(         apc_base_driver      src/apc_base_driver.cpp)
add_dependencies(       apc_base_driver      apc_msgs_generate_messages_cpp)
target_link_libraries(  apc_base_driver      ${catkin_LIBRARIES})

add_executable(         origin_tf_publisher  src/origin_tf_publisher.cpp)
add_dependencies(       origin_tf_publisher  apc_msgs_generate_messages_cpp)
target_link_libraries(  origin_tf_publisher  ${catkin_LIBRARIES})

add_executable(         print_state       	src/print_state.cpp )
add_dependencies(       print_state  		apc_msgs_generate_messages_cpp)
target_link_libraries(  print_state       	${catkin_LIBRARIES} )

add_executable(         print_joint_states       src/print_joint_states.cpp)
target_link_libraries(  print_joint_states       apc_robot_moveit ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

add_executable(         print_cartesian_pose     src/print_cartesian_pose.cpp)
target_link_libraries(  print_cartesian_pose     apc_robot_moveit ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

add_executable(         apc_arms_joint_node      src/apc_arms_joint.cpp)
target_link_libraries(  apc_arms_joint_node      ${catkin_LIBRARIES})

## Libraries
add_library(            apc_arms_cartesian        src/apc_arms_cartesian.cpp)
add_dependencies(       apc_arms_cartesian   	  apc_msgs_generate_messages_cpp)
target_link_libraries(  apc_arms_cartesian        ${catkin_LIBRARIES})

add_library(            apc_arms_joint          src/apc_arms_joint.cpp)
target_link_libraries(  apc_arms_joint          ${catkin_LIBRARIES})

add_library(            apc_robot_moveit        src/apc_robot_moveit.cpp)
target_link_libraries(  apc_robot_moveit        ${catkin_LIBRARIES})

add_library(            apc_robot_grippers      src/apc_robot_grippers.cpp)
add_dependencies(       apc_robot_grippers   	apc_msgs_generate_messages_cpp)
target_link_libraries(  apc_robot_grippers      ${catkin_LIBRARIES})

add_library(            pr2_torso        		src/pr2_torso.cpp)
#add_executable(        pr2_torso        		src/pr2_torso.cpp)
add_dependencies(       pr2_torso     	        apc_msgs_generate_messages_cpp)
target_link_libraries(  pr2_torso        		${catkin_LIBRARIES})

add_library(            pr2_head       			src/pr2_head.cpp)
#add_executable(        pr2_head       			src/pr2_head.cpp)
target_link_libraries(  pr2_head        		${catkin_LIBRARIES})

add_library(            pr2_base                src/base.cpp)
target_link_libraries(  pr2_base                ${catkin_LIBRARIES})
add_dependencies(       pr2_base                move_base_msgs_generate_messages_cpp)


## Old versions and Tesing

add_executable(         grippers     src/apc_robot_grippers.cpp)
add_dependencies(       grippers   	 apc_msgs_generate_messages_cpp)
target_link_libraries(  grippers     ${catkin_LIBRARIES})

add_executable(         testing_node           src/testing/testing_node.cpp)
target_link_libraries(  testing_node           apc_robot_moveit ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(         origin_camera_tf_publisher  src/origin_camera_tf_publisher.cpp)
#target_link_libraries(  origin_camera_tf_publisher  ${catkin_LIBRARIES})

#add_executable(         odom_pcl_limit  src/odom_pcl_limit.cpp)
#target_link_libraries(  odom_pcl_limit  ${catkin_LIBRARIES})

#add_executable(         kinecthand_tf_publisher  src/kinect_tf_publisher.cpp)
#target_link_libraries(  kinecthand_tf_publisher  ${catkin_LIBRARIES})

#add_library(            apc_robot_commander     src/apc_robot_commander.cpp)
#target_link_libraries(  apc_robot_commander     apc_robot_moveit apc_robot_grippers ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(movegroup_test src/testing/movegroup_test.cpp)
#target_link_libraries(movegroup_test ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(kinematic_model src/kinematic_model.cpp)
#target_link_libraries(kinematic_model ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(         test_video_demo        src/testing/test_video_demo.cpp)
#target_link_libraries(  test_video_demo        apc_robot_grippers apc_robot_moveit apc_robot_commander ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(         cartesian_path         src/testing/cartesian_path.cpp)
#target_link_libraries(  cartesian_path         apc_robot_moveit apc_robot_commander ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#add_executable(         pickandplace           src/testing/pickandplace.cpp)
#target_link_libraries(  pickandplace           apc_robot_grippers apc_robot_moveit apc_robot_commander ${catkin_LIBRARIES}) # ${Boost_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS apc_robot_moveit apc_robot_grippers pr2_torso pr2_head apc_arms_cartesian pr2_base apc_arms_joint
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
