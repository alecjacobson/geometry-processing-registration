#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  //R = Eigen::Matrix3d::Identity();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d O =  Eigen::Matrix3d::Zero();
  O(0,0) = 1;
  O(1,1) = 1;
  O(2,2) = (svd.matrixU() * (svd.matrixV().transpose())).determinant();
  R = svd.matrixU() * O * (svd.matrixV().transpose());
}
