#include <thread>
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Pose.h"
#include <gazebo/math/gzmath.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <stdio.h>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo {

  class RovPlugin : public ModelPlugin {
    public: RovPlugin() : ModelPlugin()
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
    }

    public: void load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  	  //TODO: set up model here
      // Store the model pointer for convenience.
      this->model = _model;
      if (!ros::isInitialized()) {
      	int argc = 0;
      	char **argv = NULL;
      	ros::init(argc, argv, "gazebo_client",
      	ros::init_options::NoSigintHandler);
      }
        // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RovPlugin::OnUpdate, this, _1));
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Pose>(
        "simulated_pose",
        1,
        boost::bind(&RovPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&RovPlugin::QueueThread, this));
    }
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Pose &_msg) {
      pos = math::Vector3(_msg.position.x,_msg.position.y, 
        _msg.position.z);
      rot = math::Quaternion(_msg.orientation.x,
      	_msg.orientation.y, _msg.orientation.z, _msg.orientation.w);
      pose = math::Pose(pos,rot);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread() {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
      // Apply a small linear velocity to the model.
      this->model->SetWorldPose(pose,false,false);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    // Variables for converting a geometry msg to data used by gazebo
    private: math::Vector3 pos;
    private: math::Quaternion rot;
    private: math::Pose pose;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RovPlugin)
}