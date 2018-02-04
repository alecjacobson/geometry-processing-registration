#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
#include "closest_rotation.h"
using namespace Eigen;
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  //Compute B
    int n = X.rows();
    Eigen::MatrixXd B;
    B = Eigen::MatrixXd::Zero(n,1);
    for (int i = 0; i < 3; i ++) {
        B.array() += N.col(i).array() * (X.col(i).array() - P.col(i).array());
    }
    
    //Compute A (in the assignment)
    Eigen::MatrixXd A, C,M;
    A = Eigen::MatrixXd::Zero(3*n, 6);
    for (int i = 3; i < 6; i ++) {
        A.block((i-3)*n,i,n,1) = Eigen::MatrixXd::Ones(n,1);
    }
    
    A.block(n,0,n,1) = -X.col(2);
    A.block(2*n,0,n,1) = X.col(1);
    
    A.block(0,1,n,1) = X.col(2);
    A.block(2*n,1,n,1) = X.col(0);
    
    A.block(0,2,n,1) = -X.col(1);
    A.block(n,2,n,1) = X.col(0);
    
    //Compute C for which CX = B;
    C = Eigen::MatrixXd::Zero(n,6);
    for (int i = 0; i < 3; i ++) {
        for (int j = 0; j < 6; j ++) {
            C.col(j).array() += N.col(i).array() * A.block(i*n,j,n,1).array();
        }
    }
    
    Eigen::RowVectorXd u;
    u = C.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
    M = Eigen::Matrix3d::Identity();
    t = u.tail(3);
    
    M(1,0) = u(2);
    M(2,0) = -u(1);
    M(0,1) = -u(2);
    M(2,1) = u(0);
    M(0,2) = u(1);
    M(1,2) = -u(0);
    
    closest_rotation(M,R);
    
}
