cmake_minimum_required(VERSION 2.8.3)
project(apc_cortex)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apc_robot
  roscpp
  rospy
  apc_msgs
  pr2_common_action_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  LIBRARIES apc_cortex
#  CATKIN_DEPENDS apc_robot roscpp rospy apc_msgs pr2_common_action_msgs actionlib
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
)

## Libraries
add_library(            apc_cortex            src/apc_cortex.cpp)
add_dependencies(		apc_cortex            apc_msgs_generate_messages_cpp)
target_link_libraries(  apc_cortex            base_movement ${catkin_LIBRARIES})

add_library(            base_movement         src/BaseMovement.cpp)
add_dependencies(		base_movement         apc_msgs_generate_messages_cpp)
target_link_libraries(  base_movement         ${catkin_LIBRARIES})

add_library(			cortex_pnp_lib        src/cortex_pick_and_place.cpp)
add_dependencies(		cortex_pnp_lib        apc_msgs_generate_messages_cpp)
add_dependencies(		cortex_pnp_lib        pr2_common_action_msgs_generate_messages_cpp)
target_link_libraries(	cortex_pnp_lib        ${catkin_LIBRARIES})

add_executable(			cortex_pnp_node       src/cortex_pnp_node.cpp)
add_dependencies(		cortex_pnp_node       apc_msgs_generate_messages_cpp)
target_link_libraries(	cortex_pnp_node       cortex_pnp_lib ${catkin_LIBRARIES})

## Executables
add_executable(         apc_cortex_node       src/apc_cortex_node.cpp)
add_dependencies(		apc_cortex_node       apc_msgs_generate_messages_cpp)
target_link_libraries(  apc_cortex_node       apc_cortex ${catkin_LIBRARIES})

add_executable(         arms_testing       src/testing/arms_cartesian_testing.cpp)
add_dependencies(		arms_testing       apc_msgs_generate_messages_cpp)
target_link_libraries(  arms_testing       apc_cortex ${catkin_LIBRARIES})


## Testing
add_executable(         base_movement_node    src/testing/base_movement_example.cpp)
add_dependencies(		base_movement_node    apc_msgs_generate_messages_cpp)
target_link_libraries(  base_movement_node    base_movement apc_cortex ${catkin_LIBRARIES})

