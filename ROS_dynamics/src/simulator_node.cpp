#include "simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ROS_INFO("Launching node simulator.");
  ros::NodeHandle nh;
  Simulator simulator(100); // Run at 100 Hz
  //ros::ServiceServer ss1 = nh.advertiseService("set_control_mode", &Controller::setControlMode, &controller);
  //ros::ServiceServer ss2 = nh.advertiseService("set_controller_gains", &Controller::setControllerGains, &controller);
  simulator.spin();
  return 0;
}