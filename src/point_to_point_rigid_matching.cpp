#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  Eigen::RowVector3d X_mean = X.colwise().mean();
  Eigen::MatrixXd X_cor = X.rowwise() - X_mean;

  Eigen::RowVector3d P_mean = P.colwise().mean();
  Eigen::MatrixXd P_cor = P.rowwise() - P_mean;

  Eigen::MatrixXd M = (P_cor.transpose() * X_cor).transpose();
  closest_rotation(M, R);
  t = (P_mean.transpose() - R*X_mean.transpose()).transpose();
  // Formula is according to column vectors
}