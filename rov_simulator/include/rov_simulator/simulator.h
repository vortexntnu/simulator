#ifndef ROV_SIMULATOR_SIMULATOR_H
#define ROV_SIMULATOR_SIMULATOR_H

#include "ros/ros.h"

#include "rov_simulator/dynamics.h"
#include "vortex_msgs/Float64ArrayStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include </usr/include/armadillo>
#include <vector>

typedef std::vector<double> stdvec;

class Simulator
{
  public:
    Simulator(unsigned int f,
              ros::NodeHandle nh);
    void thrustCallback(const vortex_msgs::Float64ArrayStamped &msg);
    void spin();
  private:
    void poseArmaToMsg(const arma::vec &e,
                       geometry_msgs::Pose &m);
    void twistArmaToMsg(const arma::vec &e,
                        geometry_msgs::Twist &m);
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

#endif  // ROV_SIMULATOR_SIMULATOR_H
