#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <Eigen/Eigen>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Fetch some useful variables
  int rows = X.rows();
  Eigen::VectorXd X1 = X.col(0);
  Eigen::VectorXd X2 = X.col(1);
  Eigen::VectorXd X3 = X.col(2);
  Eigen::VectorXd diff1 = X1 - P.col(0);
  Eigen::VectorXd diff2 = X2 - P.col(1);
  Eigen::VectorXd diff3 = X3 - P.col(2);
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd one = Eigen::VectorXd::Ones(rows);

  // Construct Matrix A and right hand side value:
  Eigen::MatrixXd A(3*rows, 6);
  A << zero, X3, -X2, one, zero, zero,
       -X3, zero, X1, zero, one, zero,
       X2, -X1, zero, zero, zero, one;

  Eigen::VectorXd rhs(3*rows);
  rhs << diff1,
         diff2,
         diff3;
  
  // Compute u:
  Eigen::MatrixXd AAT = A.transpose()*A;
  rhs = -1 * (A.transpose()*rhs);
  Eigen::VectorXd u = AAT.colPivHouseholderQr().solve(rhs);

  // Construct M:
  Eigen::Matrix3d M;
  M << 1, -u(2), u(1),
       u(2), 1, -u(0),
       -u(1), u(0), 1;

  // Calculate R and t:
  closest_rotation(M, R);
  t << u(3), u(4), u(5);
}

