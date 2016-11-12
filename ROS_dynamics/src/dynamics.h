#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "ros/ros.h"
#include <armadillo>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "vortex_msgs/ThrusterForces.h"

typedef std::vector<double> stdvec;

class Dynamics()
{
public:
  Dynamics(unsigned int frequency);

  void compute();

private:
  ros::NodeHandle nh;


  void getConfig();
  void updateCurrentNEDVelocityVector();
  void updateCoriolisMatrices();
  void updateRestoringForceVector();
  void updateCurrentPositionVector();
  void updateCurrentVelocityVector();
  void updateDampingMatrix();
  void updateRotMatrix();
  void normaliseQuaternions();
  arma::mat skewSymmetric();

  arma::vec          p;          // Position state 3D
  arma::vec          q;          // Orientation state (quaternion)
  arma::vec          euler_init; // Initial euler angle orientation
  arma::vec          eta;        // Position and orientation vector
  arma::vec          nu;         // Velocity state (linear and angular)
  arma::vec          nu_ned      // Velocity state (NED frame)
  arma::vec          nu_dot      // accelleration vector (linear and angular)
  arma::vec          tau;        // Control ROV forces

  arma::mat          T;          // Thrust config 
  arma::mat          T_pinv      // Pseudoinverse of T

  arma::vec          g;          // Restoring force vector
  arma::mat          R;          // Rotation matrix from {b} to {n}
  arma::mat          M_a;        // Added mass matrix
  arma::mat          M;          // Mass & inertia matrix
  arma::mat          C;          // Coriolis matrix
  arma::mat          D_l;        // Linear damping matrix
  arma::mat          D_q;        // Quadratic damping matrix
  arma::mat          D;          // Combined linear and quadratic damping
  arma::mat          dq;         // Quadratic damping parameters
  arma::mat          J_q         // Quaternion rotation matrix
  arma::mat          R_q         // Linear velocity rotation matrix
  arma::mat          T_q         // Angular velocity rotation matrix
  arma::mat          3x3Zero     // 3x3 zero matrix

  arma::mat          r_g;        // Center of gravity, expressed in body frame
  arma::mat          r_b;        // Center of buoyancy, expressed in body frame
  arma::vec          f_nb;       // Bouyancy force
  arma::vec          f_ng;       // Gravitational force 
  double W;                      // [N] Weight of ROV
  double B;                      // [N] Buoyancy of ROV
  double timeStep;               // Timestep in the forward euler method

  
  inline bool getMatrixParam(ros::NodeHandle nh, std::string name, arma::mat &X)
  {
    XmlRpc::XmlRpcValue matrix;
    nh.getParam(name, matrix);

    try
    {
      const int rows = matrix.size();
      const int cols = matrix[0].size();
      X.zeros(rows, cols);
      for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
          X(i,j) = matrix[i][j];
    }
    catch(...)
    {
      return false;
    }
   return true;
  }

 };
 inline double s(double val) 
 {
    return std::sin(val);
 }
 inline double c(val)
 {
    return std::cos(val);
 }
#endif