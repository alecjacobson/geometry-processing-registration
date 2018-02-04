#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iostream>
using namespace std;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
    Eigen::JacobiSVD<Eigen::Matrix3d> svdofM(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d Omega, tempVal;
    
    tempVal = svdofM.matrixV()*(svdofM.matrixU().transpose());
    
    Omega = Eigen::Matrix3d::Identity();
    Omega(2,2) = tempVal.determinant();
    R = (svdofM.matrixV() * Omega)*svdofM.matrixU().transpose();

    //cout << "M: " << M << "\n";
}
