#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::MatrixXd U, V, omega;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  U = svd.matrixU();
  V = svd.matrixV();
  omega = Eigen::Matrix3d::Identity();
  omega(2,2) = (U * V.transpose()).determinant();

  R = U * omega * V.transpose();
}
