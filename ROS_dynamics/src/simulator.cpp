#include "simulator.h"

Simulator::Simulator(unsigned int f)
{
  frequency = f;
  u = vec(8);
  dynamics = new Dynamics(f);
  wrenchSub = nh.subscribe("thruster_forces", 10, &Dynamics::thrustCallback, this);
  posePub = nh.advertise<geometry_msgs::pose>("simulated_pose", 10);
  twistPub = nh.advertise<geometry_msgs::twist>("simulated_twist", 10);
  imuPub = nh.advertise<sensor_msgs::imu>("simulated_imu",10);
  pressurePub = nh.advertise<sensor_msgs::FluidPressure>("simulated_pressure",10);
}

void Simulator::thrustCallback(const vortex_msgs::ThrusterForces &msg)
{
  u=conv_to<vec>from::(msg.thrust);
}

void Simulator::spin()
{
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    dynamics.calculate(u);
    geometry_msgs::pose posemsg = dynamics.getEta();
    geometry_msgs::twist twistmsg = dynamics.getNu();
    posePub.publish(posemsg);
    twistPub.publish(twistmsg);
    rate.sleep();
  }
}
