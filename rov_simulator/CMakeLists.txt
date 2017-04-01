cmake_minimum_required(VERSION 2.8.3)
project(rov_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vortex_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Armadillo REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
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
  ${catkin_INCLUDE_DIRS}
  ${Armadillo_INCLUDE_DIRS}
)

link_directories(/usr/include /usr/share/Armadillo)


add_executable(simulator_dynamics src/simulator_node.cpp
                          src/simulator.cpp
                          src/dynamics.cpp
)

add_dependencies(simulator_dynamics ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(simulator_dynamics ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
