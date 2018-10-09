#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <Eigen/Core>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();
  // A
  Eigen::MatrixXd A;
  A.resize(3*X.rows(),6);
  Eigen::VectorXd diag_one = Eigen::VectorXd::Ones(X.rows());
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(X.rows());
  A << zero, X.col(2), -X.col(1), diag_one, zero, zero,
       -X.col(2), zero, X.col(0), zero, diag_one, zero,
       X.col(1), -X.col(0), zero, zero, zero, diag_one;

  // X - P
  Eigen::MatrixXd X_minus_P;
  X_minus_P.resize(3*X.rows(),1);
  X_minus_P << X.col(0) - P.col(0),
               X.col(1) - P.col(1),
               X.col(2) - P.col(2);

  // u
  Eigen::Matrix<double,6,1> u = (A.transpose()*A).inverse() * (-A.transpose()*X_minus_P);

  // a,b,r
  double a, b, r;
  a = u(0,0);
  b = u(1,0);
  r = u(2,0);

  // t
  t << u(3,0),u(4,0),u(5,0);

  // M
  Eigen::Matrix3d M;
  M << 1, -r, b,
       r, 1, -a,
       -b, a, 1;

  // R
  closest_rotation(M, R);


}

