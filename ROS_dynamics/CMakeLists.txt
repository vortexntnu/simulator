cmake_minimum_required(VERSION 2.8.3)
project(ROS_dynamics)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  message_generation
  vortex_msgs
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
)

find_package(Armadillo REQUIRED)
find_package(GTest REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ResetEstimator.srv
)

generate_messages()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  cmake_modules
  roscpp
  message_generation
  eigen_conversions
  vortex_msgs
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(dynamics src/dynamics/simulator_node.cpp
                          src/dynamics/simulator.cpp
                          src/dynamics/dynamics.cpp
)


add_dependencies(controller ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(allocator  ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(estimator  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(allocator  ${catkin_LIBRARIES})
target_link_libraries(estimator  ${catkin_LIBRARIES})

#############
## Testing ##
#############

# catkin_add_gtest(controller_test              test/controller_test.cpp)
# catkin_add_gtest(depth_hold_test              test/depth_hold_test.cpp)
# catkin_add_gtest(allocator_test               test/allocator_test.cpp)
# catkin_add_gtest(integration_filter_test      test/integration_filter_test.cpp)
# catkin_add_gtest(open_loop_integration_test   test/open_loop_integration_test.cpp)
# catkin_add_gtest(closed_loop_integration_test test/closed_loop_integration_test.cpp)

# target_link_libraries(allocator_test               ${catkin_LIBRARIES})
# target_link_libraries(controller_test              ${catkin_LIBRARIES})
# target_link_libraries(depth_hold_test              ${catkin_LIBRARIES})
# target_link_libraries(integration_filter_test      ${catkin_LIBRARIES})
# target_link_libraries(open_loop_integration_test   ${catkin_LIBRARIES})
# target_link_libraries(closed_loop_integration_test ${catkin_LIBRARIES})
