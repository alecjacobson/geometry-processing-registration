#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, ComputeThinU | ComputeThinV);
  Eigen::Matrix3d omega;
  omega << 1, 0, 0,
  0, 1, 0,
  0, 0, (svd.matrixU() * svd.matrixV().transpose()).determinant();
  R = svd.matrixU() * omega * svd.matrixV().transpose();
}
