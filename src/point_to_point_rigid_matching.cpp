#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  int k = (int) X.rows();

  Eigen::VectorXd zero(k), one(k);
  zero = Eigen::VectorXd::Zero(k);
  one = Eigen::VectorXd::Ones(k);

  Eigen::MatrixXd A(3 * k, 6);
  A << zero, X.col(2), -X.col(1), one, zero, zero,
       -X.col(2), zero, X.col(0), zero, one, zero,
       X.col(1), -X.col(0), zero, zero, zero, one;

  Eigen::VectorXd xp(3 * k);
  xp << X.col(0) - P.col(0),
        X.col(1) - P.col(1),
        X.col(2) - P.col(2);

  Eigen::VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * xp);

  Eigen::Matrix3d tmp;
  tmp = Eigen::Matrix3d::Zero();
  tmp(1, 0) = u(2);
  tmp(2, 0) = -u(1);
  tmp(0, 1) = -u(2);
  tmp(2, 1) = u(0);
  tmp(0, 2) = u(1);
  tmp(1, 2) = -u(0);

  tmp += Eigen::Matrix3d::Identity();

  closest_rotation(tmp, R);

  t(0) = u(3);
  t(1) = u(4);
  t(2) = u(5);
}

