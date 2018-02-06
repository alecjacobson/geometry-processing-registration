#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
#include "closest_rotation.h"
#include <iostream>
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  int k = X.rows();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * k, 6);
  A.block(k,0, k,1) = -X.col(2);
  A.block(2*k,0, k,1) = X.col(1);

  A.block(0,1, k,1) = X.col(2);
  A.block(2*k,1, k,1) = -X.col(0);

  A.block(0,2, k,1) = -X.col(1);
  A.block(k,2, k,1) = X.col(0);

  A.block(0,3, k,1) = Eigen::MatrixXd::Ones(k,1);
  A.block(k,4, k,1) = Eigen::MatrixXd::Ones(k,1);
  A.block(k*2,5, k,1) = Eigen::MatrixXd::Ones(k,1);

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3 * k, 1);
  B.block(0,0, k,1) = X.col(0) - P.col(0);
  B.block(k,0, k,1) = X.col(1) - P.col(1);
  B.block(2*k,0, k,1) = X.col(2) - P.col(2);

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(k, 3 * k);
  C.block(0,0,k,k) = N.col(0).asDiagonal();
  C.block(0,k,k,k) = N.col(1).asDiagonal();
  C.block(0,2*k,k,k) = N.col(2).asDiagonal();

  A = C * A;
  B = C * B;

  Eigen::Matrix<double, 6, 1> u;

  u = (A.transpose() * A).inverse() * (- A.transpose() * B);
  Eigen::Matrix3d M;
  M << 1 , -u(2,0), u(1,0),
       u(2,0), 1, -u(0,0),
       -u(1,0), u(0,0), 1;
  closest_rotation(M, R);
  t << u(3,0), u(4,0), u(5,0); 
}
