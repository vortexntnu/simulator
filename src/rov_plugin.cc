#include "rov_plugin.hh"

#include <boost/bind.hpp>
#include <stdio.h>
#include <thread>

void gazebo::Rov::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  this->m_model = _parent;
  this->m_link = m_model->GetLink("link");
  this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Rov::simulationCallback, this, _1));

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client");
  }

  this->m_nh.reset(new ros::NodeHandle("gazebo_client"));
  this->m_rovForcesSub =
    this->m_nh->subscribe("/rov_forces", 10, &Rov::rovForceCallback, this);
  this->m_thrusterForcesSub =
    this->m_nh->subscribe("/thruster_forces", 10, &Rov::thrusterForceCallback, this);
  this->m_pressurePub =
    this->m_nh->advertise<sensor_msgs::FluidPressure>("/sensors/pressure",10);
  this->m_timer = this->m_nh->createTimer(ros::Duration(0.1), &Rov::timerCallback, this);
}


void gazebo::Rov::simulationCallback(const common::UpdateInfo &)
{
  this->m_pose = this->m_link->GetWorldCoGPose();

  this->applyThrusterForces();
  this->applyFrictionForces();
}


void gazebo::Rov::rovForceCallback(const geometry_msgs::Wrench &_msg)
{
  // Change sign of force in y, and of torque in y and z
  math::Vector3 forceMsg(_msg.force.x, -_msg.force.y, -_msg.force.z);
  math::Vector3 torqueMsg(_msg.torque.x, -_msg.torque.y, -_msg.torque.z);

  this->m_force = forceMsg;
  this->m_torque = torqueMsg;
}


void gazebo::Rov::thrusterForceCallback(const vortex_msgs::ThrusterForces &_msg)
{
  for (int i = 0; i < 8; i++)
    this->m_thrusterForces[i] = this->c_thrusterLayout[i]*_msg.thrust[i];
}


void gazebo::Rov::timerCallback(const ros::TimerEvent&)
{
  this->m_pressure.fluid_pressure = ((50 - this->m_pose.pos.z)*1000*9.810665 + 101300);
  this->m_pressurePub.publish(this->m_pressure);
}

void gazebo::Rov::applyFrictionForces()
{
  math::Vector3 v_l = this->m_link->GetRelativeLinearVel();
  math::Vector3 v_a = this->m_link->GetRelativeAngularVel();
  math::Vector3 F_l = this->c_linearDragCoeff*v_l*v_l.GetAbs();
  math::Vector3 F_a = this->c_angularDragCoeff*v_a*v_a.GetAbs();

  this->m_link->AddRelativeForce(-F_l);
  this->m_link->AddRelativeTorque(-F_a);
}

void gazebo::Rov::applyRovForces()
{
  this->m_link->AddRelativeForce(this->m_force);
  this->m_link->AddRelativeTorque(this->m_torque);
}

void gazebo::Rov::applyThrusterForces()
{
  double w = this->m_pose.rot.w;
  double x = this->m_pose.rot.x;
  double y = this->m_pose.rot.y;
  double z = this->m_pose.rot.z;

  math::Matrix3 R
    ((1-2*y*y-2*z*z), 2*(x*y+w*z)    , 2*(x*z-w*y),
     2*(x*y-w*z)    , (1-2*x*x-2*z*z), 2*(y*z+w*x),
     2*(x*z+w*y)    , 2*(y*z-w*x)    , (1-2*x*x-2*y*y));
  math::Matrix3 RI = R.Inverse();
  for (int i = 0; i < 8; i++)
    this->m_link->AddForceAtRelativePosition(RI*m_thrusterForces[i],
c_thrusterPosition[i]);
}
