#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "ros/ros.h"

#include "dynamics.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "vortex_msgs/ThrusterForces.h"
#include </usr/include/armadillo>

typedef std::vector<double> stdvec;

class Simulator
{
  public:
  	Simulator(unsigned int f, ros::NodeHandle nh);
  	void thrustCallback(const vortex_msgs::ThrusterForces &msg);
    void spin();
  private:
    void poseArmaToMsg(const arma::vec &e, geometry_msgs::Pose &m);
    void twistArmaToMsg(const arma::vec &e, geometry_msgs::Twist &m);
  	unsigned int    frequency;
  	ros::NodeHandle nh;
  	ros::Subscriber wrenchSub;
    ros::Publisher  posePub;
    ros::Publisher  twistPub;
    ros::Publisher  imuPub;
    ros::Publisher  pressurePub;
  	Dynamics        *dynamics;
  	arma::vec       u;
};

#endif