#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(X.rows() * 3, 6);
  Eigen::VectorXd B = Eigen::VectorXd::Zero(X.rows() * 3);
  Eigen::MatrixXd XP = X-P;
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
  B << XP.col(0), XP.col(1), XP.col(2);
  Eigen::VectorXd u = (A.transpose() * A).inverse() * (- A.transpose() * B);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
  M << 1, -u(2), u(1),
  	   u(2), 1, -u(0),
  	   -u(1), u(0), 1;
  closest_rotation(M, R);
  t << u(3), u(4), u(5);
}

