#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  Eigen::MatrixXd dig = Eigen::MatrixXd::Zero(X.rows(), X.rows() *3);
  dig.block(0,0,X.rows(),X.rows()) = N.col(0).asDiagonal();
  dig.block(0,X.rows(),X.rows(),X.rows()) = N.col(1).asDiagonal();
  dig.block(0,X.rows()*2,X.rows(),X.rows()) = N.col(2).asDiagonal();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(X.rows() * 3, 6);
  for (int i = 0; i<X.rows();i ++){
  	A(i, 1) = X(i, 2);
  	A(i, 2) = -X(i, 1);
  	A(i, 3) = 1;

  	A(i+X.rows(), 0) = -X(i, 2);
  	A(i+X.rows(), 2) = X(i, 0);
  	A(i+X.rows(), 4) = 1;

  	A(i+X.rows()*2, 0) = X(i, 1);
  	A(i+X.rows()*2, 1) = -X(i, 0);
  	A(i+X.rows()*2, 5) = 1;

  }

  Eigen::VectorXd B = Eigen::VectorXd::Zero(X.rows() * 3);
  Eigen::MatrixXd XP = X-P;
  B << XP.col(0), XP.col(1), XP.col(2);
  Eigen::MatrixXd new_A = dig * A;
  Eigen::MatrixXd new_B = dig * B;

  Eigen::VectorXd u = (new_A.transpose() * new_A).inverse() * (-new_A.transpose() * new_B);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
  M << 1, -u(2), u(1),
  	   u(2), 1, -u(0),
  	   -u(1), u(0), 1;
  closest_rotation(M, R);
  t << u(3), u(4), u(5);
  std::cout << R << std::endl;
  std::cout << t << std::endl;
}
