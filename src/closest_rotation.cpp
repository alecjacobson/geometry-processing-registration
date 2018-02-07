#include "closest_rotation.h"
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
std::cout << "ss2" << std::endl;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
std::cout << "pp2" << std::endl;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
std::cout << "tll" << std::endl;
  I(2,2) = (svd.matrixU()*svd.matrixV().transpose()).determinant();
  
  R = (svd.matrixU() * I  * svd.matrixV().transpose()).transpose();

}
