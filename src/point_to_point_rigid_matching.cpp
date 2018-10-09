#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"

// closed form solution
void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::Matrix3d M;

  // First find the average point in P and X
  Eigen::Vector3d P_avg = P.colwise().mean();
  Eigen::Vector3d X_avg = X.colwise().mean();

      


  Eigen::MatrixXd X_bar = X.rowwise() - X_avg.transpose();
  Eigen::MatrixXd P_bar = P.rowwise() - P_avg.transpose();



  M = P_bar.transpose() * X_bar;
  
  
  closest_rotation(
  M,
  R);
  

  t = (P_avg - R * X_avg).transpose();
}

