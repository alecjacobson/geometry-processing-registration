#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::RowVector3d X_mean = X.colwise().mean();
  Eigen::MatrixXd X_cor = X.rowwise() - X_mean;

  Eigen::RowVector3d P_mean = P.colwise().mean();
  Eigen::MatrixXd P_cor = P.rowwise() - P_mean;

  Eigen::MatrixXd M = P.transpose() * X;
  closest_rotation(M, R);
  t = (P_cor.transpose() - R*X_cor.transpose()).transpose();
  // Formula is according to column vectors
}