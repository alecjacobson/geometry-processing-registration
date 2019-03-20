#include "closest_rotation.h"
#include <iostream>
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d Omega;
    Omega <<    1,0,0,
                0,1,0,
                0,0,(U*V.transpose()).determinant();
    R = U*Omega*V.transpose();
}
