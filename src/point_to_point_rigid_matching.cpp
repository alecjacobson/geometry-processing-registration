#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  
  // vector of ones
  Eigen::VectorXd ones = Eigen::VectorXd::Constant(X.rows(), 1);

  // compute centroids
  Eigen::RowVector3d x_hat = X.colwise().mean();
  Eigen::RowVector3d p_hat = P.colwise().mean();

  Eigen::MatrixXd X_bar = X.rowwise() - x_hat;
  Eigen::MatrixXd P_bar = P.rowwise() - p_hat;

  // solve for R
  closest_rotation(P_bar.transpose()*X_bar, R);

  t = (p_hat.transpose() - R*x_hat.transpose()).transpose();
}

