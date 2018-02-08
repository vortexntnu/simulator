#include "mounted_rov_plugin.hh"


#include <boost/bind.hpp>
#include <stdio.h>
#include <thread>



void gazebo::Rov::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  this->model = _parent;
  this->link = model->GetLink("link");
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Rov::OnUpdate, this, _1));

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client");
  }

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  this->rosSub = this->rosNode->subscribe("/rov_forces", 10, &Rov::OnRosMsg, this);
  this->rosPub = this->rosNode->advertise<sensor_msgs::FluidPressure>("/sensors/pressure",10);
  this->rosTimer = this->rosNode->createTimer(ros::Duration(0.1), &Rov::simple_callback, this);

  this->linFrictionCoef  = math::Vector3(80,80,203);
  this->angFrictionCoef = math::Vector3(1,1,1);
  this->forceSign = math::Vector3(1,-1,1);
  this->torqueSign = math::Vector3(1,-1,-1);
}


void gazebo::Rov::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  this->link->AddRelativeForce(this->force*this->forceSign);
  this->link->AddRelativeTorque(this->torque*this->torqueSign);

  math::Vector3 linVel = this->link->GetRelativeLinearVel();
  math::Vector3 angVel = this->link->GetRelativeAngularVel();
  math::Vector3 linFriction = this->linFrictionCoef*linVel*linVel.GetAbs();
  math::Vector3 angFriction = this->angFrictionCoef*angVel*angVel.GetAbs();

  this->link->AddRelativeForce(-linFriction);
  this->link->AddRelativeTorque(-angFriction);

  this->pose = this->link->GetWorldCoGPose();
}


void gazebo::Rov::OnRosMsg(const geometry_msgs::Wrench &_msg)
{
  math::Vector3 force_msg(_msg.force.x, _msg.force.y, _msg.force.z);
  math::Vector3 torque_msg(_msg.torque.x, _msg.torque.y, _msg.torque.z);

  this->force = force_msg;
  this->torque = torque_msg;
}


void gazebo::Rov::simple_callback(const ros::TimerEvent&)
{
  this->fluidPressure.fluid_pressure = -((50 - this->pose.pos.z)*1000*9.810665 + 101300);
  this->rosPub.publish(this->fluidPressure);
}
