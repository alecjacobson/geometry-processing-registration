#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include <Eigen/Cholesky>
#include "closest_rotation.h"
void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  int k = X.rows();

  Eigen::VectorXd X1 = X.col(0);
  Eigen::VectorXd X2 = X.col(1);
  Eigen::VectorXd X3 = X.col(2);

  Eigen::VectorXd P1 = P.col(0);
  Eigen::VectorXd P2 = P.col(1);
  Eigen::VectorXd P3 = P.col(2);

  Eigen::VectorXd zero = Eigen::VectorXd::Zero(k);
  Eigen::VectorXd one = Eigen::VectorXd::Ones(k);

  Eigen::MatrixXd A(3*k, 6);
  A << zero,   X3,  -X2,  one, zero, zero,
        -X3, zero,   X1, zero,  one, zero,
         X2,  -X1, zero, zero, zero,  one;

  Eigen::VectorXd b(3*k);
  b << (X1 - P1),
       (X2 - P2),
       (X3 - P3);

  Eigen::VectorXd u = (A.transpose() * A).llt().solve(-A.transpose() * b);

  double alpha = u[0];
  double beta = u[1];
  double gamma = u[2];

  Eigen::Matrix3d M;
  M <<     1, gamma,   -beta,
       -gamma,      1, alpha,
       beta,  -alpha,      1;

  closest_rotation(M, R);
  t = u.tail(3);
}

