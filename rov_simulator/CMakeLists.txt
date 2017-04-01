cmake_minimum_required(VERSION 2.8.3)
project(rov_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  vortex_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Armadillo REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  vortex_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  DEPENDS
  Armadillo
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Armadillo_INCLUDE_DIRS}
)

link_directories(/usr/include /usr/share/Armadillo)


add_executable(simulator_dynamics src/simulator_node.cpp
                          src/simulator.cpp
                          src/dynamics.cpp
)

add_dependencies(simulator_dynamics vortex_msgs_generate_messages_cpp)

target_link_libraries(simulator_dynamics ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)
  roslint_add_test()
endif()
