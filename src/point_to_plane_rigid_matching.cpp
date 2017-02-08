#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

// Given a set of source points X and corresponding target points P and their
// normals N, find the optimal rigid transformation (R,t) that aligns X to
// planes passing through P orthogonal to N, minimizing the point-to-point
// matching energy.
//
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    static bool debug( false );
    // Replace with your code
    //R = Eigen::Matrix3d::Identity();
    //t = Eigen::RowVector3d::Zero();

    const int k( X.rows() );
    Eigen::MatrixXd A, NN;

    // Build A which is a 3k x 6
    A = Eigen::MatrixXd::Zero( 3*k, 6 );
    
    A.block( 0,   3, k, 1 ) =  Eigen::VectorXd::Ones( k );
    A.block( k,   4, k, 1 ) =  Eigen::VectorXd::Ones( k );
    A.block( 2*k, 5, k, 1 ) =  Eigen::VectorXd::Ones( k );
    // as in notes...
    A.block( 0,   1, k, 1 ) =  X.col( 2 ); //  x_i,3
    A.block( 0,   2, k, 1 ) = -X.col( 1 ); // -x_i,2
    A.block( k,   0, k, 1 ) = -X.col( 2 ); // -x_i,3
    A.block( k,   2, k, 1 ) =  X.col( 0 ); //  x_i,1

    A.block( 2*k, 0, k, 1 ) =  X.col( 1 ); //  x_i,2
    A.block( 2*k, 1, k, 1 ) = -X.col( 0 ); // -x_i,1

    if( debug )
        std::cout<<"A:"<<A<<std::endl<<std::endl;

    // Now build NN which is a k x 3
    NN.resize( k, 3*k );
    NN.setZero();
    NN.block( 0,   0, k, k ).diagonal() = N.col( 0 );
    NN.block( 0,   k, k, k ).diagonal() = N.col( 1 );
    NN.block( 0, 2*k, k, k ).diagonal() = N.col( 2 );

    if( debug )
    {
        std::cout<<"N's:"<<std::endl<<N<<std::endl; 
        std::cout<<"NN:"<<std::endl<<NN<<std::endl;
    }

    // XmP is the X1 - P1 etc, it's a 3k column vector
    Eigen::VectorXd XmP( 3*k );
    XmP.block( 0,   0, k, 1 ) = X.col( 0 ) - P.col( 0 );
    XmP.block( k,   0, k, 1 ) = X.col( 1 ) - P.col( 1 );
    XmP.block( 2*k, 0, k, 1 ) = X.col( 2 ) - P.col( 2 );
    // Eigen::VectorXd XmP(3*k);
    // XmP.segment(0*k,k) = (X-P).col(0);
    // XmP.segment(1*k,k) = (X-P).col(1);
    // XmP.segment(2*k,k) = (X-P).col(2);
    if( debug )
    {
        std::cout<<"XmP:"<<std::endl<<XmP<<std::endl;
        std::cout<<"Size Checks:"<<std::endl;
        std::cout<<"A "<<A.rows()<<"x"<<A.cols()<<std::endl;
        std::cout<<"NN "<<NN.rows()<<"x"<<NN.cols()<<std::endl;
        std::cout<<"XmP "<<XmP.rows()<<"x"<<XmP.cols()<<std::endl;
    }
    
    // Solve in the Ax = b style.
    // Here that's, NA u = - N Xmp
    // A -> NN*A
    // x -> u
    // b -> -NN*XmP
    Eigen::MatrixXd AA, b;
    AA = NN*A;
    b = -NN*XmP;
    Eigen::VectorXd u;
    u = AA.fullPivHouseholderQr().solve( b );
    //u = AA.jacobiSvd( Eigen::ComputeThinU | Eigen::ComputeThinV ).solve( b );

    // u = { alpha, beta, gamma, tx, ty, tz };
    if( debug )
        std::cout<<"u after solve: "<<u.rows()<<"x"<<u.cols()<<std::endl<<u<<
            std::endl;

    // now extract the alpha,beta,gamma into the rotation matrix
    R.setZero();
    R( 0, 1 ) = -u( 2 ); // -gamma
    R( 0, 2 ) =  u( 1 ); // beta
    R( 1, 0 ) =  u( 2 ); // gamma
    R( 1, 2 ) = -u( 0 ); // -alpha
    R( 2, 0 ) = -u( 1 ); // -beta
    R( 2, 1 ) =  u( 0 ); // alpha

    if( debug )
        std::cout<<"R: "<<std::endl<<R<<std::endl;

    Eigen::Matrix3d M;
    M = Eigen::Matrix3d::Identity() + R;
    closest_rotation( M, R );
    t = u.bottomRows( 3 );

    if( debug )
    {
        std::cout<<"M: "<<std::endl<<M<<std::endl;
        std::cout<<"closest rotation R:"<<std::endl<<R<<std::endl;
        std::cout<<"t:"<<std::endl<<t<<std::endl;
    }
}
