cmake_minimum_required(VERSION 2.8.3)
project(rov_simulator)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  message_generation
  vortex_msgs
  nav_msgs
  sensor_msgs
)

find_package(Armadillo REQUIRED)
#find_package(GTest REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


generate_messages()


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  cmake_modules
  roscpp
  message_generation
  vortex_msgs
  nav_msgs
  sensor_msgs
  DEPENDS
  Armadillo
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${Armadillo_INCLUDE_DIRS}
)

link_directories(/usr/include /usr/share/Armadillo)


add_executable(simulator_dynamics src/simulator_node.cpp
                          src/simulator.cpp
                          src/dynamics.cpp
)
#set_target_properties(simulator_dynamics PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(simulator_dynamics ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(simulator_dynamics ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})


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
