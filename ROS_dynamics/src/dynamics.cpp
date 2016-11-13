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
  3x3Zero = mat(3,3,fill:zeros);
  p = vec(3);
  q = vec(4);
  eta = vec(7);
  nu = vec(6);
  r_g = vec(3);
  r_b = vec(3);
  Dynamics::getConfig();
  if(!arma::pinv(T_pinv,T))
    ROS_ERROR("Failed to compute pseudoinverse of thrust config matrix.");
}

void Dynamics::calculate(arma::vec u)
{

  Dynamics::updateCoriolisMatrices();
  Dynamics::updateDampingMatrix();
  Dynamics::updateRestoringForceVector();
  Dynamics::updateRotMatrix();
  tau = arma::pinv(T)*u;
  nu_dot = arma::inv(M + M_a)*(tau-C*nu-D*nu-g);
  nu = nu + timeStep * nu_dot;
  eta_dot = J_q * nu;
  eta = eta + timeStep * eta_dot;
  Dynamics::normalizeQuaternions();
}

void Dynamics::getConfig()
{
  //Added mass matrix
  if (!Dynamics::getMatrixParam(nh,"/rov/addedmass", M_a))
    ROS_ERROR("Failed to read parameter rov/addedmass.");
  //Linear damping matrix
  if (!Dynamics::getMatrixParam(nh,"/rov/lineardamping", D_l))
    ROS_ERROR("Failed to read parameter rov/lineardamping.");
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
  if (!getParam("rov/quaddampingvector",dq))
    ROS_ERROR("Failed to read quadratic damping params from param server.");
  // Initial orientation
  if (!getParam("rov/eulerinitial",euler_init))
    ROS_ERROR("Failed to read initial orientation from param server.");
  // Initial position
  if (!getParam("rov/positioninitial",p))
    ROS_ERROR("Failed to read initial position from param server.");
  // Weight of the ROV, m*g
  if (!getParam("rov/weight",W))
    ROS_ERROR("Failed to read Rov mass from param server.");
  // Bouyancy of the ROV, rho*g*M³
  if (!getParam("rov/bouyancy",B))
    ROS_ERROR("Failed to read bouyancy from param server.");

  // Algorithm from p32 Fossen
  // Initialize quaternions based on a given euler angle representation
  double phi = euler_init(0);
  double theta = euler_init(1);
  double psi = euler_init(2);
  arma::mat R_euler = {{c(psi)*c(theta), -s(psi)*c(phi)+c(psi)*s(theta)*s(phi), s(psi)*s(phi)+c(psi)*s(phi)*s(theta)},
                       {s(psi)*c(theta), c(psi)*c(phi)+s(phi)*s(theta)*s(psi), -c(psi)*s(phi)+s(theta)*s(psi)*c(phi)},
                       {-s(theta), c(theta)*s(phi), c(theta)*c(phi)}};
  double R44 = trace(R_euler);
  double max = R44;
  int index = 3;

  for (int i = 0; i < R_euler.n_cols; i++)
  {
    if(R_euler(i,i) > max) {
      max = R_euler(i,i);
      index = i;
    } 
  }
  arma::vec p = vec(4);
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
  f_nb = zeros<vec>(3);
  f_ng = zeros<vec>(3);
  f_nb(2) = -B;
  f_ng(2) = W;
}

void Dynamics::updateCoriolisMatrices()
{
  //First update added mass coriolis matrix 
  arma::mat C_a = mat(6,6);
  arma:vec var1 = M_a.submat(1,1,3,3)*nu.subvec(0,2) + M_a.submat(1,4,1,6)*nu.subvec(3,6);
  arma::vec var2 = M_a.submat(4,1,4,6)*nu.subvec(0,2) + M_a.submat(4,4,6,6)*nu.subvec(3,6);
  C_a = arma::join_cols(arma::join_rows(3x3Zero, -scewsymmetrix(var1)),arma::join_rows(-skewSymmetric(var1), -skewSymmetric(var2)));

  // Update rigid body coriolis matrix
  arma::mat C_rb = mat(6,6);
  arma:vec var1 = M.submat(1,1,3,3)*nu.subvec(0,2) + M.submat(1,4,1,6)*nu.subvec(3,6);
  arma::vec var2 = M.submat(4,1,4,6)*nu.subvec(0,2) + M.submat(4,4,6,6)*nu.subvec(3,6);
  C_rb = arma::join_cols(arma::join_rows(3x3Zero, -scewsymmetric(var1)),arma::join_rows(-skewSymmetric(var1), -skewSymmetric(var2)));
  // Compute resulting coriolis force
  C = (C_a + C_rb);
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
  D = (D_l + D_q);
}

// Update current restoring force vector
void Dynamics::updateRestoringForceVector()
{
  arma::vec f_rest = arma::inv(R_q)*(f_nb + f_ng);
  arma::vec t_rest = arma::cross(r_g, arma::inv(R_q)*f_ng) + arma::cross(r_b,arma::inv(R_q)*f_nb);
  
  for (int i = 0; i < g.n_elem; i ++)
  {
    if (i < f_rest.n_elem)
    {
      g(i) = f_rest(i);
      continue;
    }
    g(i) = t_rest(i-f_rest.n_elem);
  }
  
}

// Update the rotation matrix with new values
void Dynamics::updateRotMatrix()
{
  q = eta.subvec(3,6);
  // Update rotation matrix for linear velocity
  R_q = {{1-2*(q(2)²+q(3)²), 2*(q(1)*q(2)-q(3)*q(0)), 2*(q(1)*q(3)+q(2)*q(0))},
         {2*(q(1)*q(2)+q(3)*q(0)), 1-2*(q(1)²+q(3)²), 2*(q(2)*q(3)-q(1)*q(0))},
         {2*(q(1)*q(3)-q(2)*q(0)), 2*(q(2)*q(3)+q(1)*q(0)), 1-2*(q(1)²+q(2)²)}};

  // Update rotation matrix for angular velocity
  T_q = {{-q(1), -q(2), -q(3)},
         {q(0), -q(3), -q(2)},
         {q(3), q(0), -q(1)},
         {-q(2), q(1), q(0)}};

  J_q = arma::join_cols(arma::join_rows(R_q,3x3Zero),arma::join_rows(3x3Zero,T_q));
}

void Dynamics::normaliseQuaternions() 
{
  q = arma::normalise(eta.subvec(3,6));
  for (int i = 0; i < q.n_elem; i++)
  {
    eta(i+p.n_elem) = q(i);
  }

}

// Return a scew-symmetric matrix representing cross-product operator
arma::mat Dynamics::skewSymmetric(arma::vec x)
{
  arma::mat s = {{0.0, -x(2), x(1)},
                 {x(2), 0.0, -x(0)},
                 {-x(1), x(0), 0.0}};
  return s;
}

std::vector getEta()
{
  return arma::conv_to<stdvec>::from(eta);
}

std::vector getNu()
{
  return arma::conv_to<stdvec>::from(nu);
}