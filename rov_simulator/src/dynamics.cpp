#include "dynamics.h"
#include "simulator.h"

Dynamics::Dynamics(unsigned int frequency,
                   ros::NodeHandle nh) : nh(nh) {
  timeStep = 1.0/frequency;
  // Initialize system matrices as identity
  M_a = arma::mat(6,6,arma::fill::eye);
  M = arma::mat(6,6,arma::fill::eye);
  Cor = arma::mat(6,6,arma::fill::eye);
  D_l = arma::mat(6,6,arma::fill::eye);
  D_q = arma::mat(6,6,arma::fill::eye);
  Damp = arma::mat(6,6,arma::fill::eye);
  zeros_3x3 = arma::mat(3,3,arma::fill::zeros);
  zeros_4x3 = arma::mat(4,3,arma::fill::zeros);
  R_q = arma::mat(3,3, arma::fill::zeros);
  T_q = arma::mat(4,3, arma::fill::zeros);
  p = arma::vec(3);
  q = arma::vec(4);
  eta = arma::vec(7);
  eta_dot = arma::vec(7);
  nu = arma::vec(6);
  g_vec = arma::vec(6);
  nu_dot = arma::vec(6);
  r_g = arma::vec(3);
  r_b = arma::vec(3);
  dq = arma::vec(12);
  tau6 = arma::vec(6);
  tau6(3) = 0;
  Dynamics::getConfig();
  if(!arma::pinv(T_pinv,T))
    ROS_ERROR("Failed to compute pseudoinverse of thrust config matrix.");
}

void Dynamics::calculate(arma::vec u) {
  Dynamics::updateRotMatrix();
  Dynamics::updateCoriolisMatrices();
  Dynamics::updateDampingMatrix();
  Dynamics::updateRestoringForceVector();
  
  tau = T*u;
  // Quickfix since T*u -> dim(5)
  for (int i = 0; i++; i < tau.n_elem)
  {
    if (i < 3)
    {
      tau6(i) = tau(i);
      continue;
    }
    tau(i+1) = tau(i);
  }
  nu_dot = arma::inv(M + M_a)*(tau6-Cor*nu-Damp*nu-g_vec);
  nu = nu + timeStep * nu_dot;
  eta_dot = J_q * nu;
  eta = eta + timeStep * eta_dot;
  Dynamics::normaliseQuaternions();
}

void Dynamics::getConfig() {
  //Inertia matrix
  if (!Dynamics::getMatrixParam(nh,"/physical/inertia", M))
    ROS_ERROR("Failed to read parameter physical/inertia.");
  //Added mass matrix
  if (!Dynamics::getMatrixParam(nh,"/physical/added_mass", M_a))
    ROS_ERROR("Failed to read parameter physical/added_mass.");
  //Linear damping matrix
  if (!Dynamics::getMatrixParam(nh,"/physical/linear_damping", D_l))
    ROS_ERROR("Failed to read parameter physical/linear_damping.");
  //Thruster config matrix
  if (!Dynamics::getMatrixParam(nh, "propulsion/thrusters/configuration_matrix", T))
    ROS_ERROR("Failed to read thrust config matrix from param server.");
  //Center of boyancy from CO
  if (!Dynamics::getVectorParam(nh, "physical/center_of_buoyancy", r_b))
    ROS_ERROR("Failed to read center of boyancy from param server.");
  //Center of gravity from CO
  if (!Dynamics::getVectorParam(nh,"physical/center_of_mass",r_g))
    ROS_ERROR("Failed to read centerofgravity from param server.");
  //Vector of quadratic damping parameters
  if (!Dynamics::getVectorParam(nh, "physical/quad_damping_vector",dq))
    ROS_ERROR("Failed to read quadratic damping params from param server.");
  // Initial orientation
  if (!Dynamics::getVectorParam(nh, "rov/euler_initial",euler_init))
    ROS_ERROR("Failed to read initial orientation from param server.");
  // Initial position
  if (!Dynamics::getVectorParam(nh, "rov/position_initial",p))
    ROS_ERROR("Failed to read initial position from param server.");
  // Weight of the ROV, m*g
  if (!nh.getParam("physical/mass_kg",m))
    ROS_ERROR("Failed to read Rov mass from param server.");
  // Bouyancy of the ROV, rho*g*M³
  if (!nh.getParam("physical/displacement_m3",V))
    ROS_ERROR("Failed to read bouyancy from param server.");

  W = m * g;
  B = rho * g * V;
  // Algorithm from p32 Fossen
  // Initialize quaternions based on a given euler angle representation
  double phi = euler_init(0);
  double theta = euler_init(1);
  double psi = euler_init(2);
  arma::mat R_euler = arma::mat(3,3);
  R_euler << c(psi)*c(theta) << -s(psi)*c(phi)+c(psi)*s(theta)*s(phi) << s(psi)*s(phi)+c(psi)*s(phi)*s(theta) << arma::endr
                    << s(psi)*c(theta) << c(psi)*c(phi)+s(phi)*s(theta)*s(psi) << -c(psi)*s(phi)+s(theta)*s(psi)*c(phi) << arma::endr
                    << -s(theta) << c(theta)*s(phi) << c(theta)*c(phi) << arma::endr;
  double R44 = arma::trace(R_euler);
  double max = R44;
  int index = 3;

  for (int i = 0; i < R_euler.n_cols; i++)
  {
    if(R_euler(i,i) > max) {
      max = R_euler(i,i);
      index = i;
    } 
  }
  arma::vec p = arma::vec(4);
  p(index) = abs(sqrt(1+2*max-R44));
  switch(index)
  {
    case(0):
      p(1) = (R_euler(1,0) - R_euler(0,1))/p(0);
      p(2) = (R_euler(0,2) - R_euler(2,0))/p(0);
      p(3) = (R_euler(2,1) - R_euler(1,2))/p(0);     
      break;
    case(1):
      p(0) = (R_euler(1,0) - R_euler(0,1))/p(1);
      p(2) = (R_euler(2,1) - R_euler(1,2))/p(1);
      p(3) = (R_euler(0,2) - R_euler(2,0))/p(1);            
      break;
    case(2):
      p(0) = (R_euler(0,2) - R_euler(2,0))/p(2);
      p(1) = (R_euler(2,1) - R_euler(1,2))/p(2);
      p(3) = (R_euler(1,0) - R_euler(0,1))/p(2);
      break;
    case(3):
      p(0) = (R_euler(2,1) - R_euler(1,2))/p(3);
      p(1) = (R_euler(0,2) - R_euler(2,0))/p(3);
      p(2) = (R_euler(1,0) - R_euler(0,1))/p(3);
  }
  q(0) = p(3)/2;
  q(1) = p(0)/2;
  q(2) = p(1)/2;
  q(3) = p(2)/2;
  // Initialize eta vector
  for (int i = 0; i < eta.n_elem; i++)
  {
    if(i < p.n_elem)
    {
      eta(i) = p(i);
      continue;
    }
    eta(i) = q(i-p.n_elem);
  }
  // Create bouyancy and gravitational force vector
  f_nb = arma::zeros<arma::vec>(3);
  f_ng = arma::zeros<arma::vec>(3);
  f_nb(2) = -B;
  f_ng(2) = W;
}

void Dynamics::updateCoriolisMatrices() {
  //First update added mass coriolis matrix 
  arma::mat C_a = arma::mat(6,6);
  arma::vec var1 = M_a.submat(0,0,2,2)*nu.subvec(0,2) + M_a.submat(0,3,2,5)*nu.subvec(3,5);
  arma::vec var2 = M_a.submat(3,0,5,2)*nu.subvec(0,2) + M_a.submat(3,3,5,5)*nu.subvec(3,5);
  C_a = arma::join_cols(arma::join_rows(zeros_3x3, -Dynamics::skewSymmetric(var1)),arma::join_rows(-Dynamics::skewSymmetric(var1), -Dynamics::skewSymmetric(var2)));

  // Update rigid body coriolis matrix
  arma::mat C_rb = arma::mat(6,6);
  var1 = M.submat(0,0,2,2)*nu.subvec(0,2) + M.submat(0,3,2,5)*nu.subvec(3,5);
  var2 = M.submat(3,0,5,2)*nu.subvec(0,2) + M.submat(3,3,5,5)*nu.subvec(3,5);
  C_rb = arma::join_cols(arma::join_rows(zeros_3x3, -Dynamics::skewSymmetric(var1)),arma::join_rows(-Dynamics::skewSymmetric(var1), -Dynamics::skewSymmetric(var2)));
  // Compute resulting coriolis force
  Cor = (C_a + C_rb);
}

void Dynamics::updateDampingMatrix() {
  // Update nonlinear damping matrix
  D_q << dq(0)*abs(nu(0)) << 0 << 0 << 0 << 0 << 0 << arma::endr
      << 0 << dq(1)*abs(nu(1)) + dq(2)*abs(nu(5)) << 0 << 0 << dq(3)*abs(nu(1)) + dq(4)*abs(nu(5)) << arma::endr
      << 0 << 0 << dq(5)*abs(nu(2)) << 0 << 0 << 0 << arma::endr
      << 0 << 0 << 0 << dq(6)*abs(nu(3)) << 0 << 0 << arma::endr
      << 0 << 0 << 0 << 0 << dq(7)*abs(nu(4)) << 0 << arma::endr
      << 0 << dq(8)*abs(nu(1)) + dq(9)*abs(nu(5)) << 0 << 0 << 0 << dq(10)*abs(nu(1)) + dq(11)*abs(nu(5)) << arma::endr;
  // Compute total damping force
  Damp = (D_l + D_q);
}

// Update current restoring force vector
void Dynamics::updateRestoringForceVector() {
  arma::vec f_rest = arma::inv(R_q)*(f_nb + f_ng);
  arma::vec t_rest = arma::cross(r_g, arma::inv(R_q)*f_ng) + arma::cross(r_b,arma::inv(R_q)*f_nb);
  
  for (int i = 0; i < g_vec.n_elem; i ++)
  {
    if (i < f_rest.n_elem)
    {
      g_vec(i) = f_rest(i);
      continue;
    }
    g_vec(i) = t_rest(i-f_rest.n_elem);
  }
  
}

// Update the rotation matrix with new values
void Dynamics::updateRotMatrix() {
  q = eta.subvec(3,6);
  // Update rotation matrix for linear velocity
  R_q << 1-2*(pow(q(2),2)+pow(q(3),2)) << 2*(q(1)*q(2)-q(3)*q(0)) << 2*(q(1)*q(3)+q(2)*q(0)) << arma::endr
      << 2*(q(1)*q(2)+q(3)*q(0)) << 1-2*(pow(q(1),2)+pow(q(3),2)) << 2*(q(2)*q(3)-q(1)*q(0)) << arma::endr
      << 2*(q(1)*q(3)-q(2)*q(0)) << 2*(q(2)*q(3)+q(1)*q(0)) << 1-2*(pow(q(1),2)+pow(q(2),2)) << arma::endr;

  // Update rotation matrix for angular velocity
  T_q << -q(1) << -q(2) << -q(3) << arma::endr
      << q(0) << -q(3) << -q(2) << arma::endr
      << q(3) << q(0) << -q(1) << arma::endr
      << -q(2) << q(1) << q(0) << arma::endr;

  J_q = arma::join_cols(arma::join_rows(R_q,zeros_3x3),arma::join_rows(zeros_4x3,T_q));
}

void Dynamics::normaliseQuaternions() {
  q = arma::normalise(eta.subvec(3,6));
  for (int i = 0; i < q.n_elem; i++)
  {
    eta(i+p.n_elem) = q(i);
  }

}

// Return a scew-symmetric matrix representing cross-product operator
arma::mat Dynamics::skewSymmetric(arma::vec x) {
  arma::mat s = arma::mat(3,3);
  s << 0.0 << -x(2) << x(1) << arma::endr
    << x(2) << 0.0 << -x(0) << arma::endr
    << -x(1) << x(0) << 0.0 << arma::endr;
  return s;
}

arma::vec Dynamics::getEta() {
  return eta;
}

arma::vec Dynamics::getNu() {
  return nu;
}