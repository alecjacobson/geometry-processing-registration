#include "point_to_plane_rigid_matching.h"
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  int k = X.rows();
  // Create u = [alpha, beta, gamma, tx, ty, tz]
  Eigen::VectorXd u(6);
  // Create N_diag
  Eigen::MatrixXd N_diag = Eigen::MatrixXd::Zero(k, 3*k);
  N_diag.block(0,0,k,k).diagonal() = N.col(0);
  N_diag.block(0,k,k,k).diagonal() = N.col(1);
  N_diag.block(0,2*k,k,k).diagonal() = N.col(2);

  // Create A
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*k, 6);
  // Fill A with 1 columns
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(k);
  A.block(0,3,k,1) = ones;
  A.block(k,4,k,1) = ones;
  A.block(2*k,5,k,1) = ones;
  // Fill A with columns of X
  A.block(0,1, k, 1) = X.col(2);
  A.block(0,2, k, 1) = -1 * X.col(1);
  A.block(k, 0, k, 1) = -1 * X.col(2);
  A.block(k, 2, k, 1) = X.col(0);
  A.block(2*k, 0, k, 1) = X.col(1);
  A.block(2*k, 1, k, 1) = -1 * X.col(0);

  // Get X - P
  Eigen::VectorXd X_minus_P(3*k);
  X_minus_P.segment(0,k) = X.col(0) - P.col(0);
  X_minus_P.segment(k,k) = X.col(1) - P.col(1);
  X_minus_P.segment(2*k,k) = X.col(2) - P.col(2);

  // Solve for u
  //u = ((NA)^T(NA))^-1 * [-1 * (NA)^T * (NXP)]
  Eigen::MatrixXd NA = N_diag*A;
  Eigen::MatrixXd NA_T = NA.transpose();
  u = (NA_T*NA).inverse() * (-1 * NA_T * (N_diag*X_minus_P));
  // Recover R and update t
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d delta;
  delta << 0, -u[2], u[1], u[2], 0, -u[0], -u[1], u[0], 0;
  M += delta;
  closest_rotation(M, R);
  t = Eigen::RowVector3d(u[3], u[4], u[5]);
}
