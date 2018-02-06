#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  Eigen::Matrix3d Omega = Eigen::Matrix3d::Zero();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto temp = svd.matrixU() *  (svd.matrixV().transpose());
  Omega(2,2) = temp.determinant();
  Omega(0,0) = 1;
  Omega(1,1) = 1;



//according to this https://github.com/alecjacobson/geometry-processing-registration/issues/21
  R = svd.matrixV() * Omega * svd.matrixU().transpose();
}
