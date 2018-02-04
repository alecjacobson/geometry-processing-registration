#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V_T = svd.matrixV().transpose();
  Eigen::Matrix3d O = Eigen::Matrix3d::Identity();
  O(2,2) = (U * V_T).determinant();
  R = U * O * V_T;
}
