#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <stdlib.h>

#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/FluidPressure.h"

namespace gazebo
{
  class Rov : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
      this->model = _parent;
      this->link = model->GetLink("link");
      this->inertial = link->GetInertial();
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

      this->linFrictionCoef  = math::Vector3(80,80,203);
      this->angFrictionCoef = math::Vector3(1,1,1);
      this->forceSign = math::Vector3(1,-1,1);
      this->torqueSign = math::Vector3(1,-1,-1);
    }


    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
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


    public: void OnRosMsg(const geometry_msgs::Wrench &_msg)
    {
      math::Vector3 force_msg(_msg.force.x, _msg.force.y, _msg.force.z);
      math::Vector3 torque_msg(_msg.torque.x, _msg.torque.y, _msg.torque.z);

      this->force = force_msg;
      this->torque = torque_msg;

      bool mode = this->link->GetGravityMode();
/*
      ROS_INFO("Force [%f,%f,%f], Torque [%f,%f,%f]",force.x, force.y, force.z,
torque.x, torque.y, torque.z);
      math::Vector3 force_debug = this->link->GetRelativeForce();
      math::Vector3 torque_debug = this->link->GetRelativeTorque();

      ROS_INFO("DBG_F [%f, %f, %f], DBG_T [%f,%f,%f]", force_debug.x,
force_debug.y, force_debug.z, torque_debug.x, torque_debug.y, torque_debug.z);
*/

      this->fluidPressure.fluid_pressure = (50 - this->pose.pos.z)*1000*9.810665 + 101300;
      this->rosPub.publish(this->fluidPressure);
      // ADVERTISE fluidPressure here!
      std::cout << "[x,y,z , r,p,y] = " << this->pose << ", z = " <<
this->pose.pos.z << std::endl;
    }


    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::InertialPtr inertial;
    private: event::ConnectionPtr updateConnection;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::Publisher rosPub;

    private: math::Pose pose;

    private: sensor_msgs::FluidPressure fluidPressure;
//    private: geometry_msgs::SlettMeg;

    private: math::Vector3 force;
    private: math::Vector3 torque;

    private: math::Vector3 linFrictionCoef;
    private: math::Vector3 angFrictionCoef;

    private: math::Vector3 forceSign;
    private: math::Vector3 torqueSign;
  };

  GZ_REGISTER_MODEL_PLUGIN(Rov)
}
