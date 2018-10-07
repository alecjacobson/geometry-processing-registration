#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

using namespace Eigen;

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // construct diag matrix
  MatrixXd diag_matrix(X.rows(), 3 * X.rows());
  diag_matrix << MatrixXd(N.col(0).asDiagonal()),
                 MatrixXd(N.col(1).asDiagonal()),
                 MatrixXd(N.col(2).asDiagonal());

  // construct b = X - P
  VectorXd b(3 * N.rows());
  b << X.col(0) - P.col(0),
       X.col(1) - P.col(1),
       X.col(2) - P.col(2);
  b = diag_matrix * b;

  // construct A
  VectorXd zeros = VectorXd::Zero(N.rows());
  VectorXd ones = VectorXd::Ones(N.rows());
  MatrixXd A(3 * N.rows(), 6);
  A << zeros, X.col(2), -X.col(1), ones, zeros, zeros,
       -X.col(2), zeros, X.col(0), zeros, ones, zeros,
       X.col(1), -X.col(0), zeros, zeros, zeros, ones;
  A = diag_matrix * A;

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