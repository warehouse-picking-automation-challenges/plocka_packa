cmake_minimum_required(VERSION 2.8.3)
project(apc_pcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  apc_msgs
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
#  LIBRARIES apc_pcl
#  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs tf apc_msgs pcl_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_seg src/example_seg.cpp)
target_link_libraries(example_seg ${catkin_LIBRARIES})

add_executable(example_voxelgrid src/example_voxelgrid.cpp)
target_link_libraries(example_voxelgrid ${catkin_LIBRARIES})

add_executable(passthrough_filter src/passthrough_filter.cpp)
target_link_libraries(passthrough_filter ${catkin_LIBRARIES})
add_dependencies(passthrough_filter apc_msgs_generate_messages_cpp)


add_executable(alignment src/alignment.cpp)
target_link_libraries(alignment ${catkin_LIBRARIES})
add_dependencies(alignment apc_msgs_generate_messages_cpp)

add_executable (apc_shelf_detect src/apc_shelf_detect.cpp)
target_link_libraries (apc_shelf_detect ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable (tracking_sample src/tracking_sample.cpp)
target_link_libraries (tracking_sample ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable (openni_tracking src/openni_tracking.cpp)
target_link_libraries (openni_tracking ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS apc_pcl apc_pcl_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_apc_pcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
