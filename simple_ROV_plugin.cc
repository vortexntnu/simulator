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
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointers of the model
      this->model = _parent;
      this->link = model->GetLink("link");
      this->inertial = link->GetInertial();
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client");
      }
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node.
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      // Create a named topic, and subscribe to is.
      this->rosSub = this->rosNode->subscribe("/rov_forces", 10, &ModelPush::OnRosMsg, this);

      this->linFrictionCoef  = math::Vector3(80,80,203);
      this->angFrictionCoef = math::Vector3(1,1,1);

      this->forceSign = math::Vector3(1,-1,1);
      this->torqueSign = math::Vector3(1,-1,-1);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // This is runned each simulation iteration.

      this->link->AddRelativeForce(this->force*this->forceSign);
      this->link->AddRelativeTorque(this->torque*this->torqueSign);

      math::Vector3 linVel = this->link->GetRelativeLinearVel();
      math::Vector3 angVel = this->link->GetRelativeAngularVel();
      math::Vector3 linFriction = this->linFrictionCoef*linVel*linVel.GetAbs();
      math::Vector3 angFriction = this->angFrictionCoef*angVel*angVel.GetAbs();

      this->link->AddRelativeForce(-linFriction);
      this->link->AddRelativeTorque(-angFriction);
    }

    // \brief Handle an incoming message from ROS
    // \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Wrench &_msg)
    {
      // Load msg to proper types
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

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to link
    private: physics::LinkPtr link;
    // Pointer to inertia
    private: physics::InertialPtr inertial;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    // Force & Torque from rov_force topic
    private: math::Vector3 force;
    private: math::Vector3 torque;

    // Gains & Coefficients
    private: math::Vector3 linFrictionCoef;
    private: math::Vector3 angFrictionCoef;

    private: math::Vector3 forceSign;
    private: math::Vector3 torqueSign;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
