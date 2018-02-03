#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
#include <iostream>
// Given a set of source points `X` and corresponding target points `P` and
// normals `N`, find the optimal rigid transformation (`R`,`t`) that aligns `X`
// to `P`, minimizing the matching energy:
//
//   (R X' - t' 1' - P')^2
//
// Inputs:
//   X  #X by 3 set of source points
//   P  #X by 3 set of target points
//   N  #X by 3 set of target normals
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
//   
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  
  const int k = X.rows();
  Eigen::MatrixXd A(3*k, 6);
  Eigen::MatrixXd one = Eigen::MatrixXd::Ones(k, 1);
  A.block(0, 1, k, 1 ) = X.col(2);
  A.block(0, 2, k, 1 ) = -X.col(1);
  A.block(0, 3, k, 1 ) = one;

  A.block(k, 0, k, 1) = -X.col(2);
  A.block(k, 2, k, 1) = X.col(0);
  A.block(k, 4, k, 1) = one;

  A.block(2 * k, 0, k, 1 ) = X.col(1);
  A.block(2 * k, 1, k, 1 ) = -X.col(0);
  A.block(2 * k, 5, k, 1 ) = one;
  
  Eigen::MatrixXd N_blk(k, 3*k);
  N_blk.block(0, 0, k,k) = N.col(0).asDiagonal();
  N_blk.block(0, k,  k, k) = N.col(1).asDiagonal();
  N_blk.block(0, 2*k, k, k) = N.col(2).asDiagonal();
  
  Eigen::VectorXd D(3 * k);
  D.segment(0, k) = X.col(0) - P.col(0);
  D.segment(k, k) = X.col(1) - P.col(1);
  D.segment(2*k, k) = X.col(2) - P.col(2);

  
  //from https://math.stackexchange.com/questions/725185/minimize-a-x-b
  Eigen::MatrixXd A1 = N_blk*A;
  Eigen::MatrixXd b1 = -N_blk*D;

  Eigen::MatrixXd A2 =  A1.transpose()*A1;
  Eigen::VectorXd b2 =  A1.transpose()*b1;

  //from https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  Eigen::VectorXd u = A2.colPivHouseholderQr().solve(b2);
  std::cout << u << std::endl;
  R(0, 1) = -u(2); R(1, 0) =  u(2);
  R(0, 2) =  u(1); R(2, 0) = -u(1);
  R(1, 2) = -u(0); R(2, 1) =  u(0);
  std::cout << R << std::endl;
  t = u.tail(3);
  std::cout << t << std::endl;
  system("pause");
}
