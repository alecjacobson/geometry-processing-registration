#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  // R = Eigen::Matrix3d::Identity();

  // Compute the SVD of the given matrix M

  // exact solver - slow
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

  
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  double det_uv = (U * V.transpose()).determinant();

  Eigen::Matrix3d Omega = Eigen::Matrix3d::Identity();
  Omega(2,2) = det_uv;

  R = U * Omega * V.transpose();

}
