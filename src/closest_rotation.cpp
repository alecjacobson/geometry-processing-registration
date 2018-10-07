#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense> 

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d U=svd.matrixU();
  Eigen::Matrix3d V=svd.matrixV();
  Eigen::Matrix3d omega=Eigen::Matrix3d::Identity();
  R=U*V.transpose();
  if (R.determinant()<0) omega(2,2)=-1;
  R=(U*omega*V.transpose()).transpose();
}
