#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <stdlib.h>

#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

namespace gazebo
{
  class MountedRov : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      ROS_INFO("MountedRov plugin success!");
      this->model = _parent;
      this->link = model->GetLink("link");
      this->inertial = link->GetInertial();
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MountedRov::OnUpdate, this, _1));
      // LEFT CLAW IS 0, RIGHT CLAW IS 1
      this->claws = link->GetChildJointsLinks()[0]->GetChildJoints();


      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client");
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      this->rosSub_wrench = this->rosNode->subscribe("/rov_forces", 10, &MountedRov::OnRosMsg, this);
      this->rosSub_manipulator =
this->rosNode->subscribe("/manipulator_command", 10,
&MountedRov::OnManipulatorCommand)

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

      if()
    }

    public: void OnRosMsg(const geometry_msgs::Wrench &_msg)
    {
      math::Vector3 force_msg(_msg.force.x, _msg.force.y, _msg.force.z);
      math::Vector3 torque_msg(_msg.torque.x, _msg.torque.y, _msg.torque.z);

      this->force = force_msg;
      this->torque = torque_msg;

      ROS_INFO("Force [%f,%f,%f], Torque [%f,%f,%f]",force.x, force.y, force.z,
torque.x, torque.y, torque.z);
      math::Vector3 force_debug = this->link->GetRelativeForce();
      math::Vector3 torque_debug = this->link->GetRelativeTorque();

      ROS_INFO("DBG_F [%f, %f, %f], DBG_T [%f,%f,%f]", force_debug.x,
force_debug.y, force_debug.z, torque_debug.x, torque_debug.y, torque_debug.z);
    }

    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::InertialPtr inertial;
    private: physics::Link_V armBase;
    private: physics::Joint_V claws;
    private: event::ConnectionPtr updateConnection;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;


    private: math::Vector3 force;
    private: math::Vector3 torque;

    private: math::Vector3 linFrictionCoef;
    private: math::Vector3 angFrictionCoef;

    private: math::Vector3 forceSign;
    private: math::Vector3 torqueSign;
  };

  GZ_REGISTER_MODEL_PLUGIN(MountedRov)
}
