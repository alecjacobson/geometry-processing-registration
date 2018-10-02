#include "point_to_plane_rigid_matching.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  int k = X.rows();

  // Set A to zeros
  Eigen::MatrixXd A;
  A = Eigen::MatrixXd::Zero(3*k, 6);

  // Build A
  // Put all the equations corresponding to the rows
  // together, making it easy to construct A
  A.col(0).segment(k, k) = -X.col(2);
  A.col(0).segment(2*k, k) = X.col(1);
  A.col(1).segment(0, k) = X.col(2);
  A.col(1).segment(2*k, k) = -X.col(0);
  A.col(2).segment(0, k) = -X.col(1);
  A.col(2).segment(k, k) = X.col(0);
  A.col(3).segment(0, k) = Eigen::VectorXd::Ones(k);
  A.col(4).segment(k, k) = Eigen::VectorXd::Ones(k);
  A.col(5).segment(2*k, k) = Eigen::VectorXd::Ones(k);

  // Build X-P
  Eigen::VectorXd PX;
  PX = Eigen::VectorXd::Zero(3*k, 1);
  PX.segment(0, k) = X.col(0) - P.col(0);
  PX.segment(k, k) = X.col(1) - P.col(1);
  PX.segment(2*k, k) = X.col(2) - P.col(2);

  // Build N_diag
  Eigen::MatrixXd N_diag;
  N_diag = Eigen::MatrixXd::Zero(k, 3*k);
  N_diag.block(0,0,k,k) = N.col(0).asDiagonal();
  N_diag.block(0,k,k,k) = N.col(1).asDiagonal();
  N_diag.block(0,2*k,k,k) = N.col(2).asDiagonal();
  
  A = N_diag * A;
  PX = N_diag * PX;

  // Least squares solution
  Eigen::Matrix3d M, residual;
  Eigen::VectorXd u;
  u = (A.transpose() * A).inverse() * (-A.transpose() * PX);
  t = Eigen::RowVector3d(u(3),u(4),u(5));
  
  residual << 0, -u(2), u(1),
       u(2), 0, -u(0),
       -u(1), u(0), 0;

  M = Eigen::Matrix3d::Identity() - residual;
  closest_rotation(M, R);
}
