#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d Omega = (svd.matrixU() * svd.matrixV().transpose());
  R(2,2) = Omega.determinant();
  R = svd.matrixU() * R * svd.matrixV().transpose();
}
