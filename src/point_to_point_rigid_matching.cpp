#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/LU>

#include <iostream>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    const int k = X.rows();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * k, 6);

    A.block(0, 1, k, 1) = X.col(2);
    A.block(0, 2, k, 1) = -X.col(1);
    A.block(0, 3, k, 1) = Eigen::MatrixXd::Ones(k, 1);
    A.block(k, 0, k, 1) = -X.col(2);
    A.block(k, 2, k, 1) = X.col(0);
    A.block(k, 4, k, 1) = Eigen::MatrixXd::Ones(k, 1);
    A.block(2 * k, 0, k, 1) = X.col(1);
    A.block(2 * k, 1, k, 1) = -X.col(0);
    A.block(2 * k, 5, k, 1) = Eigen::MatrixXd::Ones(k, 1);

    Eigen::VectorXd x_minus_p(3 * k);
    x_minus_p.segment(0, k) = X.col(0) - P.col(0);
    x_minus_p.segment(k, k) = X.col(1) - P.col(1);
    x_minus_p.segment(2 * k, k) = X.col(2) - P.col(2);

    Eigen::VectorXd u(6);
    u = (A.transpose() * A).inverse() * (-A.transpose() * x_minus_p);

    const double alpha = u(0);
    const double beta = u(1);
    const double gamma = u(2);

    // This is different from the README; see issue #9
    Eigen::Matrix3d M;
    M << 1, gamma, -beta,
         -gamma, 1, alpha,
         beta, -alpha, 1;

    closest_rotation(M, R);
    t << u(3), u(4), u(5);
}

