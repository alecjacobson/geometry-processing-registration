#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // the closest transformation comes from the SVD decomp
  // M = UΣVᵀ
  // closest rotation is defined as:
  //       / 1    0    0     \
  // R = V | 0    1    0     | Uᵀ
  //       \ 0    0  det(UV) /
  //
  // R = V        Ω            Uᵀ     
  Eigen::Matrix3d U, Omega, V;
  
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  
  Omega << 1, 0, 0,
           0, 1, 0,
           0, 0, (V*U.transpose()).determinant();
  R = V*Omega*U.transpose();
}
