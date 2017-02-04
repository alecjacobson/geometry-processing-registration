#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d vut = svd.matrixV() * svd.matrixU().transpose();
  double detVU = vut.determinant();
  Eigen::Matrix3d sigma;
  sigma << 1, 0, 0,
           0, 1, 0,
           0, 0, detVU;
  R = svd.matrixV() * sigma * svd.matrixU().transpose();
}
