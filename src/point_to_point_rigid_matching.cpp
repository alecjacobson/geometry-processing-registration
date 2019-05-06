#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>
#include <iostream>

using namespace Eigen;

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // construct b = X - P
  VectorXd b(3 * X.rows());
  b << X.col(0) - P.col(0),
       X.col(1) - P.col(1),
       X.col(2) - P.col(2);

  // construct A
  VectorXd zeros = VectorXd::Zero(X.rows());
  VectorXd ones = VectorXd::Ones(X.rows());
  MatrixXd A(3 * X.rows(), 6);
  A << zeros, X.col(2), -X.col(1), ones, zeros, zeros,
       -X.col(2), zeros, X.col(0), zeros, ones, zeros,
       X.col(1), -X.col(0), zeros, zeros, zeros, ones;

  // compute u
  VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * b);

  double alpha = u(0);
	double beta = u(1);
	double gamma = u(2);

  Matrix3d M;
	M << 1, gamma, -beta,
		   -gamma, 1, alpha,
		   beta, -alpha, 1;

  closest_rotation(M, R);
  t = RowVector3d(u(3),u(4),u(5));
  
}


