#include "closest_rotation.h"
#include <Eigen/SVD>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, ComputeFullU | ComputeFullV);
    Eigen::Matrix3d Omega;
    
    Omega = Eigen::Matrix3d::Identity();
    Omega(3,3) = (svd.matrixU()*svd.matrixV().transpose()).determinant();
    R = (svd.matrixU() * Omega)*svd.matrixV().transpose();
}
