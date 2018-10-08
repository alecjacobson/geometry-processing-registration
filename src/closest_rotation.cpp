#include <Eigen/SVD>
#include <Eigen/LU>
#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::Matrix3d sigma = Eigen::Matrix3d::Identity();
  sigma(2, 2) = (U * V.transpose()).determinant();
  R = U * sigma * V.transpose();
}

