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
  // Construct matrix A containing points 
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * X.rows(), 6);
  Eigen::VectorXd b(3 * X.rows());
  A.block(0, 1, X.rows(), 1) = -X.col(2);
  A.block(0, 2, X.rows(), 1) = X.col(1);
  A.block(0, 3, X.rows(), 1) = Eigen::MatrixXd::Constant(X.rows(), 1, 1);
  A.block(X.rows(), 0, X.rows(), 1) = X.col(2);
  A.block(X.rows(), 2, X.rows(), 1) = -X.col(0);
  A.block(X.rows(), 4, X.rows(), 1) = Eigen::MatrixXd::Constant(X.rows(), 1, 1);
  A.block(2 * X.rows(), 0, X.rows(), 1) = -X.col(1);
  A.block(2 * X.rows(), 1, X.rows(), 1) = X.col(0);
  A.block(2 * X.rows(), 5, X.rows(), 1) = Eigen::MatrixXd::Constant(X.rows(), 1, 1);

  // Construct matrix D of normals
  Eigen::MatrixXd D(X.rows(), 3 * X.rows());
  Eigen::MatrixXd D_1 = N.col(0).asDiagonal();
  Eigen::MatrixXd D_2 = N.col(1).asDiagonal();
  Eigen::MatrixXd D_3 = N.col(2).asDiagonal();
  D.block(0, 0, X.rows(), X.rows()) = D_1;
  D.block(0, X.rows(), X.rows(), X.rows()) = D_2;
  D.block(0, 2 * X.rows(), X.rows(), X.rows()) = D_3;

  // Construct vector P -X
  b << P.col(0) - X.col(0),
       P.col(1) - X.col(1),
       P.col(2) - X.col(2);

  // Solve for the least squares solution of ||DAx - D(P-X)||^2. 
  A = D * A;
  b = D * b;
  Eigen::MatrixXd L = A.transpose() * A;
  Eigen::VectorXd r = A.transpose() * b;
  Eigen::VectorXd u = L.fullPivLu().solve(r);

  // Extract R and t from solution of optimization problem
  Eigen::Matrix3d M;
  M << 1, -u(2), u(1), 
      u(2), 1, -u(0),
      -u(1), u(0), 1;
  closest_rotation(M, R);
  t = u.tail(3);
}
