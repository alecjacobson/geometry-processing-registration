#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  const int k = X.rows();
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  Eigen::MatrixXd ND(k,3*k);



  ND.block(0,0*k,k,k).diagonal() = N.col(0);
  ND.block(0,1*k,k,k).diagonal() = N.col(1);
  ND.block(0,2*k,k,k).diagonal() = N.col(2);

  Eigen::MatrixXd A(3*k,6);

  /*

  A.block(0*k,1,k,1) = X.col(3);//3
  A.block(0*k,2,k,1) =-X.col(2);//-2
  A.block(1*k,0,k,1) =-X.col(3);//-3
  A.block(1*k,2,k,1) = X.col(1);//1
  A.block(2*k,0,k,1) = X.col(2);//2
  A.block(2*k,1,k,1) =-X.col(1);//-1
  */

  A.block(3*k,0,k,1).setConstant(1);
  A.block(4*k,1,k,1).setConstant(1);
  A.block(5*k,2,k,1).setConstant(1);


  Eigen::VectorXd b(3*k);

  b.segment(0*k,k) = (X-P).col(0);
  b.segment(1*k,k) = (X-P).col(1);
  b.segment(2*k,k) = (X-P).col(2);


  A = N * A;
  b = N * b;

  Eigen::VectorXd u = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  R.setZero();
  R(2,1) = u(0);
  R(0,2) = u(1);
  R(1,0) = u(2);
  Eigen::Matrix3d M = R - R.transpose() + Eigen::Matrix3d::Identity();
  closest_rotation(M,R);

  std::cout << "Rotation determinant: " << R.determinant() << std::endl;
  t = u.bottomRows(3);
  std::cout << "Solution vec:" << u.transpose()  << std::endl;

}
