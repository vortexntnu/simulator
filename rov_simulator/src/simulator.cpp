#include "rov_simulator/simulator.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"

Simulator::Simulator(unsigned int f, ros::NodeHandle nh) : nh(nh), frequency(f)
{
  u = arma::vec(6);
  dynamics = new Dynamics(f, nh);
  wrenchSub = nh.subscribe("thruster_forces", 10, &Simulator::thrustCallback, this);
  posePub = nh.advertise<geometry_msgs::Pose>("simulated_pose", 10);
  twistPub = nh.advertise<geometry_msgs::Twist>("simulated_twist", 10);
  imuPub = nh.advertise<sensor_msgs::Imu>("simulated_imu", 10);
  pressurePub = nh.advertise<sensor_msgs::FluidPressure>("simulated_pressure", 10);
}

void Simulator::thrustCallback(const vortex_msgs::Float64ArrayStamped &msg)
{
  u = arma::conv_to<arma::vec>::from(msg.data);
}

void Simulator::spin()
{
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    dynamics->calculate(u);
    geometry_msgs::Pose posemsg;
    arma::vec pose = dynamics->getEta();
    Simulator::poseArmaToMsg(pose, posemsg);

    geometry_msgs::Twist twistmsg;
    arma::vec twist = dynamics->getNu();
    Simulator::twistArmaToMsg(twist, twistmsg);

    posePub.publish(posemsg);
    twistPub.publish(twistmsg);
    rate.sleep();
  }
}

void Simulator::poseArmaToMsg(const arma::vec &e,
                              geometry_msgs::Pose &m)
{
  m.position.x = e(0);
  m.position.y = e(1);
  m.position.z = e(2);
  m.orientation.x = e(3);
  m.orientation.y = e(4);
  m.orientation.z = e(5);
  m.orientation.w = e(6);
}

void Simulator::twistArmaToMsg(const arma::vec &e,
                               geometry_msgs::Twist &m)
{
  m.linear.x = e(0);
  m.linear.y = e(1);
  m.linear.z = e(2);
  m.angular.x = e(3);
  m.angular.y = e(4);
  m.angular.z = e(5);
}
