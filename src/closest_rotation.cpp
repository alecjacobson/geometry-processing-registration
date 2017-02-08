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
    //MatrixXf m = MatrixXf::Random(3,2);
    //std::cout << "Here is the matrix M:" << std::endl << M << std::endl;

    //JacobiSVD<Eigen::MatrixXd> svd( M, ComputeFullU | ComputeFullV );
    JacobiSVD<Eigen::MatrixXd> svd( M, ComputeThinU | ComputeThinV );
    // std::cout << "Sigma: "<<std::endl<< svd.singularValues()<<std::endl;
    // std::cout<<"R:"<<std::endl<<svd.matrixU()<<std::endl;
    // std::cout<<"V^T:"<<std::endl<<svd.matrixV() << std::endl;
    
    Eigen::MatrixXd Sigma = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV(); // svd.matrixV().transpose() ????

    Eigen::MatrixXd Omega = Eigen::MatrixXd::Identity( 3, 3 );
    Omega(2,2) = ( U*V.transpose() ).determinant();
    //std::cout << "(U*VT).determinant(): " << (U*V.transpose()).determinant() << std::endl;
    
    R = ( U * Omega * V.transpose() ).transpose();
    //std::cout << "R:"<<std::endl<<R<<std::endl;
    //std::cout<<"det R: "<<R.determinant()<<std::endl;
    //std::cout<<"R with VT transposed:"<<std::endl<<(U*Omega*V.transpose())<<std::endl;
    //std::cout<<"R with VT transposed det:"<<(U*Omega*V.transpose()).determinant()<<std::endl;
}
