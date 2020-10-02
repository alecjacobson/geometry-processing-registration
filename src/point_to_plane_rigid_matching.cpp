#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Eigen>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
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
  Eigen::MatrixXd N1 = N.col(0).asDiagonal();
  Eigen::MatrixXd N2 = N.col(1).asDiagonal();
  Eigen::MatrixXd N3 = N.col(2).asDiagonal();
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd one = Eigen::VectorXd::Ones(rows);

  // Construct Matrix A and right hand side value:
  Eigen::MatrixXd left_N(rows, 3*rows);
  left_N << N1, N2, N3;

  Eigen::MatrixXd A(3*rows, 6);
  A << zero, X3, -X2, one, zero, zero,
       -X3, zero, X1, zero, one, zero,
       X2, -X1, zero, zero, zero, one;
  Eigen::MatrixXd new_A = left_N * A;

  Eigen::VectorXd rhs(3*rows);
  rhs << diff1,
         diff2,
         diff3;
  Eigen::VectorXd new_rhs = left_N * rhs;
  
  // Compute u:
  Eigen::MatrixXd AAT = new_A.transpose()*new_A;
  new_rhs = -1 * (new_A.transpose()*new_rhs);
  Eigen::VectorXd u = AAT.colPivHouseholderQr().solve(new_rhs);

  // Construct M:
  Eigen::Matrix3d M;
  M << 1, -u(2), u(1),
       u(2), 1, -u(0),
       -u(1), u(0), 1;

  // Calculate R and t:
  closest_rotation(M, R);
  t << u(3), u(4), u(5);
}
