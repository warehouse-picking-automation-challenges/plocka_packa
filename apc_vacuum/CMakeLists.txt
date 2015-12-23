cmake_minimum_required(VERSION 2.8.3)
project(apc_vacuum)

## Find catkin macros and libraries
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ValveCtrl.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)

include_directories(

  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(serial_read src/serial_read.cpp)
target_link_libraries(serial_read ${catkin_LIBRARIES})

#add_executable(comm src/comm.cpp)

