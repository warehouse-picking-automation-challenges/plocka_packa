cmake_minimum_required(VERSION 2.8.3)
project(apc_object_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  tf
  object_recognition_core
  object_recognition_msgs
  geometry_msgs
  visualization_msgs
  apc_msgs
  sensor_msgs
  image_transport
  cv_bridge
  std_msgs
  dynamic_reconfigure
)

# Dynamic reconfigure stuff
generate_dynamic_reconfigure_options(
  cfg/bin_filter.cfg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
 
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc_object_detection
#  CATKIN_DEPENDS object_recognition_msgs roscpp rospy geometry_msgs visualization_msgs actionlib tf apc_msgs object_recognition_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${CURL_INCLUDE_DIRS}
)

## Libraries
add_library(			apc_object_detection		src/apc_object_detection.cpp)
add_dependencies(		apc_object_detection        apc_msgs_generate_messages_cpp)
add_dependencies(       apc_object_detection        ${PROJECT_NAME}_gencfg)
target_link_libraries(	apc_object_detection		${catkin_LIBRARIES})

## Executables
add_executable(			opencv_filtering_node	    src/opencv_filtering.cpp)
add_dependencies(		opencv_filtering_node   	apc_msgs_generate_messages_cpp)
target_link_libraries(	opencv_filtering_node		${catkin_LIBRARIES})

add_executable(			apc_object_detection_node	src/apc_object_detection_node.cpp)
add_dependencies(		apc_object_detection_node   apc_msgs_generate_messages_cpp)
target_link_libraries(	apc_object_detection_node	apc_object_detection ${catkin_LIBRARIES})

add_executable(			apc_ork_manager     		src/apc_ork_manager.cpp)
add_dependencies(		apc_ork_manager		     	apc_msgs_generate_messages_cpp)
target_link_libraries(	apc_ork_manager				${catkin_LIBRARIES})

add_executable(			test_client_node     		src/testing/test_client.cpp)
add_dependencies(		test_client_node		    apc_msgs_generate_messages_cpp)
target_link_libraries(	test_client_node			apc_object_detection ${catkin_LIBRARIES})

## Testing
add_executable(			bin_pcl_filtering_node	    src/testing/bin_pcl_filtering.cpp)
add_dependencies(		bin_pcl_filtering_node   	apc_msgs_generate_messages_cpp)
target_link_libraries(	bin_pcl_filtering_node		apc_object_detection ${catkin_LIBRARIES})

add_executable(			bin_pcl_filtering_single	src/testing/bin_pcl_filtering_single.cpp)
add_dependencies(		bin_pcl_filtering_single   	apc_msgs_generate_messages_cpp)
target_link_libraries(	bin_pcl_filtering_single    apc_object_detection ${catkin_LIBRARIES})

add_executable(			opencv_filtering_test	    src/testing/bin_opencv_filtering.cpp)
add_dependencies(		opencv_filtering_test   	apc_msgs_generate_messages_cpp)
target_link_libraries(	opencv_filtering_test		${catkin_LIBRARIES})
