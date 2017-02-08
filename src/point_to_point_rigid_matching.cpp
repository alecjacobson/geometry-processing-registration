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
    XX = X.rowwise() - Xc;
    PP = P.rowwise() - Pc;

    Eigen::Matrix3d M;
    M = PP.transpose() * XX;

    // svd this...
    closest_rotation( M, R );


    // calculate t given R
    t = Pc - (R.transpose()*Xc.transpose()).transpose();
}
