cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  gazebo
  message_generation
)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${gemoetry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rov_plugin SHARED src/rov_plugin.cc)
add_library(mounted_rov_plugin SHARED src/mounted_rov_plugin.cc)
