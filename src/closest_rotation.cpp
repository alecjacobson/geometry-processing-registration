#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M,Eigen::ComputeThinU | Eigen::ComputeThinV);

  double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  Eigen::Matrix3d omega = Eigen::Matrix3d::Identity();
  omega(2,2) = det;

  R = svd.matrixU() * omega * svd.matrixV().transpose();


}
