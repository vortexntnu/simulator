#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "ros/ros.h"

#include "dynamics.h"
#include "sensor_msgs/Imu.h"

class Simulator()
{
  public:
  	Simulator(unsigned int f);
  	void thrustCallback(const geometry_msgs::wrench &msg)
    void spin();
  private:
  	unsigned int    frequency;
  	ros::NodeHandle nh;
  	ros::Subscriber thrustSub;
    ros::Publisher  posePub;
    ros::Publisher  twistPub;
    ros::Publisher  imuPub;
    ros::Publisher  pressurePub;
  	Dynamics        *dynamics;
  	arma::vec       u;
};

#endif