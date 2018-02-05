#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(const Eigen::Matrix3d & M, Eigen::Matrix3d & R) {
  
  // compute the singular value decomposition of M to recover the constituent
  // U and V matrices
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  
  // construct Omega using U and V. Omega is equal to 0 everywhere off the diagonal,
  // 1 along the diagonal except for the bottom right corner, and the determinant
  // of VU^T in the bottom right corner
  Eigen::Matrix3d Omega = Eigen::Matrix3d::Identity();
  Omega(2,2) = (V*U.transpose()).determinant();
                    
  // construct R using U, V and Omega
  R = V*Omega*U.transpose();
}
