#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdlib.h>

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/FluidPressure.h"

namespace gazebo
{
  class Rov : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr);
      void OnUpdate(const common::UpdateInfo &);
      void OnRosMsg(const geometry_msgs::Wrench &_msg);
      void simple_callback(const ros::TimerEvent&);
    private:
      physics::ModelPtr                 model;
      physics::LinkPtr                  link;
      event::ConnectionPtr              updateConnection;

      std::unique_ptr<ros::NodeHandle>  rosNode;
      ros::Subscriber                   rosSub;
      ros::Publisher                    rosPub;
      ros::Timer                        rosTimer;

      math::Pose                        pose;

      sensor_msgs::FluidPressure fluidPressure;

      math::Vector3                     force;
      math::Vector3                     torque;

      math::Vector3                     linFrictionCoef;
      math::Vector3                     angFrictionCoef;

      math::Vector3                     forceSign;
      math::Vector3                     torqueSign;
  };
  GZ_REGISTER_MODEL_PLUGIN(Rov)
}
