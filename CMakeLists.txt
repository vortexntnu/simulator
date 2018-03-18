cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  vortex_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${gemoetry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${vortex_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rov_plugin SHARED src/rov_plugin.cc)
add_library(mounted_rov_plugin SHARED src/mounted_rov_plugin.cc)

add_dependencies(rov_plugin ${catkin_EXPORTED_TARGETS})
