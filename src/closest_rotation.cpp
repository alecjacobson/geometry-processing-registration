#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d omega;
  omega << 1, 0, 0,
  0, 1, 0,
  0, 0, (svd.matrixU() * svd.matrixV().transpose()).determinant();
  R = svd.matrixV() * omega * svd.matrixU().transpose();
}
