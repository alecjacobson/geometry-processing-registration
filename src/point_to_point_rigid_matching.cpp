#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Find p_hat and x_hat
  Eigen::RowVector3d p_hat, x_hat;
  x_hat = X.colwise().sum() / X.rows();
  p_hat = P.colwise().sum() / P.rows();
  
  // Find P_bar and X_bar
  Eigen::MatrixXd X_bar(X.rows(), 3), P_bar(P.rows(), 3);
  X_bar = X - x_hat.replicate(1, X.rows());
  P_bar = P - p_hat.replicate(1, X.rows());
  
  // Find M
  Eigen::Matrix3d M;
  M = X_bar.transpose() * P_bar;
  
  // Find R
  closest_rotation(M, R);
  
  // Find t (remember that x_hat and p_hat are *row* vectors)
  t(0) = p_hat(0) - R.row(0).dot(x_hat);
  t(1) = p_hat(1) - R.row(1).dot(x_hat);
  t(2) = p_hat(2) - R.row(2).dot(x_hat);
}

