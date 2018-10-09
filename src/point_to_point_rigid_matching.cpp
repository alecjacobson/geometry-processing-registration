#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <iostream>

void point_to_point_rigid_matching(
    const Eigen::MatrixXd &X,
    const Eigen::MatrixXd &P,
    Eigen::Matrix3d &R,
    Eigen::RowVector3d &t)
{
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(X.rows());
  Eigen::VectorXd zeros = Eigen::VectorXd::Zero(X.rows());

  Eigen::VectorXd B(X.rows() * 3);
  B << X.col(0) - P.col(0), X.col(1) - P.col(1), X.col(2) - P.col(2);

  Eigen::MatrixXd A(3 * X.rows(), 6);
  A << zeros, X.col(2), -X.col(1), ones, zeros, zeros,
      -X.col(2), zeros, X.col(0), zeros, ones, zeros,
      X.col(1), -X.col(0), zeros, zeros, zeros, ones;

  Eigen::VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * B);

  Eigen::Matrix3d M;
  M << 1, -u(2), u(1),
      u(2), 1, -u(0),
      -u(1), u(0), 1;

  closest_rotation(M, R);

  t << u(3), u(4), u(5);
}
