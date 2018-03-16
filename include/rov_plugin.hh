#ifndef ROV_PLUGIN_HH
#define ROV_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "ros/ros.h"

#include "vortex_msgs/ThrusterForces.h"
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
      void thrusterForceCallback(const vortex_msgs::ThrusterForces &_msg);
      void timerCallback(const ros::TimerEvent&);
      void applyFrictionForces();
      void applyRovForces();
      void applyThrusterForces();
    private:
      physics::ModelPtr                 m_model;
      physics::LinkPtr                  m_link;
      event::ConnectionPtr              m_updateConnection;

      std::unique_ptr<ros::NodeHandle>  m_nh;
      ros::Subscriber                   m_rovForcesSub;
      ros::Subscriber                   m_thrusterForcesSub;
      ros::Publisher                    m_pressurePub;
      ros::Timer                        m_timer;

      math::Pose                        m_pose;

      sensor_msgs::FluidPressure        m_pressure;

      math::Vector3                     m_force;
      math::Vector3                     m_torque;

      std::vector<math::Vector3>        m_thrusterForces = {
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0),
      math::Vector3(0, 0, 0)};

      // ENU Convension (gazebo world reference frame)
      const std::vector<math::Vector3>  c_thrusterLayout = {
      math::Vector3( 0.70711,  0.70711,  0),
      math::Vector3( 0.00000,  0.00000, -1),
      math::Vector3( 0.00000,  0.00000, -1),
      math::Vector3(-0.70711,  0.70711,  0),
      math::Vector3(-0.70711, -0.70711,  0),
      math::Vector3( 0.00000,  0.00000, -1),
      math::Vector3( 0.00000,  0.00000, -1),
      math::Vector3( 0.70711, -0.70711,  0)};
      // ENU Convension (gazebo world reference frame)
      const std::vector<math::Vector3>  c_thrusterPosition = {
      math::Vector3( 0.7, -0.7, 0),
      math::Vector3( 0.5, -0.5, 0),
      math::Vector3(-0.5, -0.5, 0),
      math::Vector3(-0.7, -0.7, 0),
      math::Vector3(-0.7,  0.7, 0),
      math::Vector3(-0.5,  0.5, 0),
      math::Vector3( 0.5,  0.5, 0),
      math::Vector3( 0.7,  0.7, 0)};
      const math::Vector3               c_linearDragCoeff = math::Vector3(80, 80, 204);
      const math::Vector3               c_angularDragCoeff = math::Vector3(1, 1, 1);

  };
  GZ_REGISTER_MODEL_PLUGIN(Rov)
}

#endif  // ROV_PLUGIN_HH
