#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/FluidPressure.h"

namespace gazebo
{
  class Rov : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr);
      void simulationCallback(const common::UpdateInfo &);
      void rovForceCallback(const geometry_msgs::Wrench &_msg);
      void timerCallback(const ros::TimerEvent&);
    private:
      physics::ModelPtr                 m_model;
      physics::LinkPtr                  m_link;
      event::ConnectionPtr              m_updateConnection;

      std::unique_ptr<ros::NodeHandle>  m_nh;
      ros::Subscriber                   m_rovForcesSub;
      ros::Publisher                    m_pressurePub;
      ros::Timer                        m_timer;

      math::Vector3                     m_pose;

      sensor_msgs::FluidPressure        m_pressure;

      math::Vector3                     m_force;
      math::Vector3                     m_torque;

      const math::Vector3               c_linearDragCoeff = math::Vector3(80, 80, 204);
      const math::Vector3               c_angularDragCoeff = math::Vector3(1, 1, 1);

  };
  GZ_REGISTER_MODEL_PLUGIN(Rov)
}
