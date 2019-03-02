#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::Matrix3d sigma;
  sigma << 1, 0 , 0,
           0, 1 , 0,
           0, 0, (U * V.transpose()).determinant();
  R = U * sigma * V.transpose();
}
