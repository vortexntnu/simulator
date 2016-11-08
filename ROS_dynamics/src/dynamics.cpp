#include "dynamics.h"
#include "simulator.h"

Dynamics::Dynamics(unsigned int frequency)
{
  timeStep = 1.0/frequency;
  // Initialize system matrices as identity
  // Fix read from yaml
  M_a = mat(6,6,fill::eye);
  M = mat(6,6,fill::eye);
  C = mat(6,6,fill::eye);
  D_l = mat(6,6,fill::eye);
  D_q = mat(6,6,fill::eye);
  p = vec(6);
  q = vec(7);
  nu = vec(6);
  r_g = vec(3);
  r_b = vec(3);
  Dynamics::getConfig();
  if(!pinv(T_pinv,T)); // Do something if pseudoinverse is fucked
}

void Dynamics::calculate(arma::vec u)
{

  Dynamics::updateCoriolisMatrices();
  Dynamics::updateDampingMatrix();
  Dynamics::updateRestoringForceVector();
  tau = pinv(T)*u;
  nu_dot = arma::inv(M + M_a)*(tau-C-D-g);
  nu = nu + timeStep * nu_dot;
  // TODO - rotationMatrix, quarternion from nu to eta_dot
  eta = eta + timeStep * eta_dot;

void Dynamics::getConfig()
{
  //Added mass matrix
  if (!Dynamics::getMatrixParam(nh,"/rov/addedmass", M_a))
    ROS_FATAL("Failed to read parameter rov/addedmass.");
  //Linear damping matrix
  if (!Dynamics::getMatrixParam(nh,"/rov/lineardamping", D_l))
    ROS_FATAL("Failed to read parameter rov/lineardamping.");
  //Thruster config matrix
  if (!Dynamics::getMatrixParam(nh, "thrusters/configuration_matrix", T))
    ROS_ERROR("Failed to read thrust config matrix from param server.");
  //Center of boyancy from CO
  if (!nh.getParam("rov/centerofboyancy", r_b))
    ROS_ERROR("Failed to read center of boyancy from param server.");
  //Center of gravity from CO
  if (!getParam("rov/centerofgravity",r_g))
    ROS_ERROR("Failed to read centerofgravity from param server.");
  //Vector of quadratic damping parameters
  if (!getParam("rov/quaddampingvector",qd))
    ROS_ERROR("Failed to read quadratic damping params from param server.");
}

void Dynamics::updateCurrentNEDVelocityVector()
{
  // Test test
}
void Dynamics::updateCoriolisMatrices()
{
  //First update added mass coriolis matrix
  arma::mat zero = mat(3,3,fill:zeros)  
  arma::mat C_a = mat(6,6);
  arma:vec var1 = M_a.submat(1,1,3,3)*nu.subvec(0,2) + M_a.submat(1,4,1,6)*nu.subvec(3,6);
  arma::vec var2 = M_a.submat(4,1,4,6)*nu.subvec(0,2) + M_a.submat(4,4,6,6)*nu.subvec(3,6);
  arma::mat C_a1 = join_rows(zero, -scewsymmetrix(var1))
  arma::mat C_a2 = join_rows(-skewSymmetric(var1), -skewSymmetric(var2));
  C_a=join_cols(C_a1,C_a2);

  // Update rigid body coriolis matrix
  arma::mat C_rb = mat(6,6);
  arma:vec var1 = M.submat(1,1,3,3)*nu.subvec(0,2) + M.submat(1,4,1,6)*nu.subvec(3,6);
  arma::vec var2 = M.submat(4,1,4,6)*nu.subvec(0,2) + M.submat(4,4,6,6)*nu.subvec(3,6);
  arma::mat C_rb1 = join_rows(zero, -scewsymmetric(var1))
  arma::mat C_rb2 = join_rows(-skewSymmetric(var1), -skewSymmetric(var2));
  C_rb=join_cols(C_rb1,C_rb2);
  // Compute resulting coriolis force
  C = (C_a + C_rb) * nu;
}

void Dynamics::updateDampingMatrix()
{
  // Update nonlinear damping matrix
  D_q = {{dq(0)*abs(nu(0)), 0, 0, 0, 0, 0},
  {0, dq(1)*abs(nu(1)) + dq(2)*abs(nu(5)), 0, 0, dq(3)*abs(nu(1)) + dq(4)*abs(nu(5))}
  {0, 0, dq(5)*abs(nu(2)), 0, 0, 0}
  {0, 0 , 0 ,dq(6)*abs(nu(3)), 0, 0}
  {0, 0, 0, 0, dq(7)*abs(nu(4), 0)}
  {0, dq(8)*abs(nu(1)) + dq(9)*abs(nu(5)), 0, 0, 0, dq(10)*abs(nu(1)) + dq(11)*abs(nu(5))}};
  // Compute total damping force
  D = (D_l + D_q) * nu;
}
void Dynamics::updateRestoringForceVector()
{
  
}

arma::mat Dynamics::skewSymmetric(arma::vec x)
{
  arma::mat s = mat(3,3);
  s(0,0)=0.0;
  s(0,1)=-x(2);
  s(0,2)=x(1);
  s(1,0)=x(2);
  s(1,1)=0.0;
  s(1,2)=-x(0);
  s(2,0)=-x(1);
  s(2,1)=x(0);
  s(2,2)=0.0;
  return s;
}