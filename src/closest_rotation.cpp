#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
  Matrix3d omega;
  omega.setZero();
  omega(0, 0) = 1;
  omega(1, 1) = 1;
  omega(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  //std::cout<<"det: "<<omega(2,2)<<std::endl;
  R = svd.matrixU() * omega * svd.matrixV().transpose();
}
