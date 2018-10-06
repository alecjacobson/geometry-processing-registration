#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/LU>
using namespace Eigen;

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  /*
  // this is the closed-form version of the point-to-point minimizer
  Eigen::RowVector3d x_h = X.colwise().mean();
  Eigen::RowVector3d p_h = P.colwise().mean();
  Eigen::Matrix3d M = (P.rowwise() - p_h).transpose() * (X.rowwise() - x_h);
  closest_rotation(M, R);
  t = (p_h.transpose() - R * x_h.transpose()).transpose(); */
  
  int k = X.rows();
  MatrixXd A(3*k, 6);
  VectorXd a(3*k);
  VectorXd zeros = VectorXd::Zero(k);
  VectorXd ones = VectorXd::Ones(k);
  A << zeros, X.col(2), -X.col(1), ones, zeros, zeros,
      -X.col(2), zeros, X.col(0), zeros, ones, zeros,
       X.col(1), -X.col(0), zeros, zeros, zeros, ones;
  a << X.col(0) - P.col(0), 
       X.col(1) - P.col(1), 
       X.col(2) - P.col(2);

  VectorXd u = (A.transpose() * A).lu().solve(-A.transpose() * a);
  
  Matrix3d M;
  
  // I am not sure why I needed to flip the signs here
  M << 1, u(2), -u(1),
       -u(2), 1, u(0),
       u(1), -u(0), 1;  
  closest_rotation(M, R);
  
  t = u.tail(3);
}

