#include "simulator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "simulator");
  ROS_INFO("Launching node simulator.");
  ros::NodeHandle nh;
  Simulator simulator(100, nh); // Run at 100 Hz
  simulator.spin();
  return 0;
}