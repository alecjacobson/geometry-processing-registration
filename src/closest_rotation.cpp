#include "closest_rotation.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense> 
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    // Replace with your code
    R = Eigen::Matrix3d::Identity();
    
    // Given M, decompose into R sigma V
    using namespace Eigen;

    //JacobiSVD<Eigen::MatrixXd> svd( M, ComputeFullU | ComputeFullV );
    JacobiSVD<Eigen::MatrixXd> svd( M, ComputeThinU | ComputeThinV );
    
    Eigen::MatrixXd Sigma = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV(); // svd.matrixV().transpose() ????

    Eigen::MatrixXd Omega = Eigen::MatrixXd::Identity( 3, 3 );
    Omega(2,2) = ( U*V.transpose() ).determinant();
    
    R = ( U * Omega * V.transpose() ).transpose();
}
