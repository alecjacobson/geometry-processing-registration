#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>
#include <iostream>

using namespace std;

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  // Compute centroid
  Eigen::MatrixXd xcentroid = X.colwise().mean();
  
  Eigen::MatrixXd X_ = X - xcentroid.replicate(X.rows(), 1);

  Eigen::MatrixXd pcentroid = P.colwise().mean();
  Eigen::MatrixXd P_ = P - pcentroid.replicate(P.rows(), 1);
  Eigen::Matrix3d M = P_.transpose() * X_;
  closest_rotation(M, R);
  t = pcentroid - (R * xcentroid.transpose()).transpose();
  Eigen::Matrix3d RT = R.transpose();
  R = RT;


}

