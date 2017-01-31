#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d uvt = svd.matrixU() * svd.matrixV().transpose();
  double detUV = uvt.determinant();
  Eigen::Matrix3d sigma;
  sigma << 1, 0, 0,
           0, 1, 0,
           0, 0, detUV;
  R = svd.matrixU() * sigma * svd.matrixV().transpose();
}
