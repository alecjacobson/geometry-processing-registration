#include <Eigen/SVD>
#include <Eigen/LU>

#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code

  // SVD to compute U, V where M = USV^T
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV | Eigen::ComputeFullU);
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV | Eigen::ComputeThinU);

  // Compute O (Omega)
  Eigen::Matrix3d O = Eigen::Matrix3d::Identity();
  O(2,2) = (svd.matrixU()*svd.matrixV().transpose()).determinant();

  R = svd.matrixU()*O*svd.matrixV().transpose();
}
