#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <assert.h>
//Given a set of source points X and corresponding target points P, find the
// optimal rigid transformation (R,t) that aligns X to P, minimizing the
// point-to-point matching energy.
//
void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  //R = Eigen::Matrix3d::Identity();
  //t = Eigen::RowVector3d::Zero();

    // uh, should have just used .colwise().sum()
     const int nP( P.rows() );
    // Eigen::RowVector3d Pc = (1./nP) * Eigen::RowVector3d(
    //     P.block( 0, 0, nP, 1 ).sum(),
    //     P.block( 0, 1, nP, 1 ).sum(),
    //     P.block( 0, 2, nP, 1 ).sum() );

     const int nX( P.rows() );
    // Eigen::RowVector3d Xc = (1./nX) * Eigen::RowVector3d(
    //     X.block( 0, 0, nX, 1 ).sum(),
    //     X.block( 0, 1, nX, 1 ).sum(),
    //     X.block( 0, 2, nX, 1 ).sum() );
    Eigen::RowVector3d Pc = (1./nP) * P.colwise().sum();
    Eigen::RowVector3d Xc = (1./nX) * X.colwise().sum();
    
    assert( nP == nX );

    Eigen::MatrixXd XX, PP;
    XX.resizeLike( X );
    PP.resizeLike( P );

    XX = X.rowwise() - Xc;
    PP = P.rowwise() - Pc;
    // std::cout<<"X:"<<std::endl<<X<<std::endl;
    // std::cout<<"Xc:"<<std::endl<<Xc<<std::endl;
    // std::cout<<"XX:"<<std::endl<<XX<<std::endl;

    // Eigen::VectorXd Ones;
    // Ones = Eigen::VectorXd::Constant( nP, 1 );
    // std::cout<<"Ones:"<<Ones<<std::endl;
    //t = ( (PP.transpose() * Ones) - (R*XX 

    Eigen::MatrixXd M;
    M = PP * XX.transpose();

    // svd this...
    closest_rotation( M, R );

    // check sizes:
    // std::cout<<"size R: "<<R.rows()<<"x"<<R.cols()<<std::endl;
    // std::cout<<"size Xc: "<<Xc.rows()<<"x"<<Xc.cols()<<std::endl;
    // std::cout<<"size Pc: "<<Pc.rows()<<"x"<<Pc.cols()<<std::endl;
    // std::cout<<"size t: "<<t.rows()<<"x"<<t.cols()<<std::endl;

    // calculate t given R
    t = Pc - (R*Xc.transpose()).transpose();

    std::cout<<"R:"<<std::endl<<R<<std::endl;
    std::cout<<"t:"<<std::endl<<t<<std::endl;
}

// Eigen::Matrix4d RR = Eigen::Matrix4d::Identity();
// RR.block(0,0,3,3) = R;
// Eigen::Vector4d Pxx = Eigen::Vector4d::Zero();
// Pxx.block(0,0,3,1) = Xc.transpose();
// Eigen::Vector4d rotatedPoint = RR*Pxx;
// std::cout<<"Pxx:"<<std::endl<<Pxx<<std::endl;
// std::cout<<"rotatedPoint:"<<std::endl<<rotatedPoint<<std::endl;
// std::cout<<"alt rotatedPoint:"<<std::endl<<(R*Xc.transpose())<<std::endl;
// Eigen::RowVector3d finalX = rotatedPoint.transpose().block(0,0,1,3);
// t = Pc - finalX;
