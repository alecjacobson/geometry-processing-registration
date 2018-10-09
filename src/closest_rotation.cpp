#include "closest_rotation.h"
#include <Eigen/Eigen>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // SVD inspired by Eigen::JacobiSVD library
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, 
          Eigen::ComputeFullU | Eigen::ComputeFullV);
  
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  double det = (U * V.transpose()).determinant();

  Eigen::Matrix3d mid;
  mid << 1, 0, 0,
         0, 1, 0,
         0, 0, det;

  // As ISSUE #38 poited out, should times Vx by R.T
  // Fix here since I cannot change main.cpp
  R = (U * mid * V.transpose()).transpose();
}
