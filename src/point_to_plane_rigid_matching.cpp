#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();
  // nT
  Eigen::MatrixXd nT;
  nT.resize(X.rows(),3*X.rows());
  nT << Eigen::MatrixXd (N.col(0).asDiagonal()),
        Eigen::MatrixXd (N.col(1).asDiagonal()),
        Eigen::MatrixXd (N.col(2).asDiagonal());

  // A
  Eigen::MatrixXd A;
  A.resize(3*X.rows(),6);
  Eigen::VectorXd diag_one = Eigen::VectorXd::Ones(X.rows());
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(X.rows());
  A << zero, X.col(2), -X.col(1), diag_one, zero, zero,
       -X.col(2), zero, X.col(0), zero, diag_one, zero,
       X.col(1), -X.col(0), zero, zero, zero, diag_one;
  A = nT * A;

  // X - P
  Eigen::VectorXd X_minus_P;
  X_minus_P.resize(3*X.rows());
  X_minus_P << X.col(0) - P.col(0),
               X.col(1) - P.col(1),
               X.col(2) - P.col(2);
  X_minus_P = nT * X_minus_P;

  // u
  Eigen::Matrix<double,6,1> u = ((A.transpose()*A).inverse()) * (-A.transpose()*X_minus_P);

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
