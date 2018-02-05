#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{  
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, 
    Eigen::DecompositionOptions::ComputeFullU | 
    Eigen::DecompositionOptions::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  double det = (U * V.transpose()).determinant();
  
  Eigen::Matrix3d new_eigen = Eigen::Matrix3d::Identity();  
  new_eigen(2, 2) = det;
  // We need to respect the orientation, so change the least important eigen vector
  if (det < 0.0)
    new_eigen(2, 0) = -1.0;
  
  R = U * new_eigen * V.transpose();
}
