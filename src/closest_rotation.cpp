#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>


void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
    Eigen::JacobiSVD<Eigen::Matrix3d> svdofM(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d Omega, tempVal;
    
    tempVal = svdofM.matrixU()*(svdofM.matrixV().transpose());
    
    Omega = Eigen::Matrix3d::Identity();
    Omega(2,2) = tempVal.determinant();
    R = (svdofM.matrixU() * Omega)*svdofM.matrixV().transpose();
}
