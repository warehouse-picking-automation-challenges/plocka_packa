cmake_minimum_required(VERSION 2.8.3)
project(apc_marker)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  apc_msgs
  ar_track_alvar
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc_marker
#  CATKIN_DEPENDS roscpp std_msgs apc_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Executables
add_executable(			apc_marker_node 	src/apc_marker.cpp)
add_dependencies(		apc_marker_node 	apc_msgs_generate_messages_cpp)
target_link_libraries(	apc_marker_node	 	${catkin_LIBRARIES} )
