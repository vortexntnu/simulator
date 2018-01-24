cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Simulator)

find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})


# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build plugins
add_library(rov_plugin SHARED ${PROJECT_SOURCE_DIR}/plugins/rov_plugin.cc)
add_library(mounted_rov_plugin SHARED ${PROJECT_SOURCE_DIR}/plugins/mounted_rov_plugin.cc)
