#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  
  // Construct matrix A and vector P - X
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

  b << P.col(0) - X.col(0),
       P.col(1) - X.col(1),
       P.col(2) - X.col(2);

  // Solve for the least squares solution u, which minimizes ||Au - (P - X)||^2. 
  Eigen::MatrixXd L = A.transpose() * A;
  b = A.transpose() * b;
  Eigen::VectorXd u = L.fullPivLu().solve(b);

  // Extract the new R and t
  Eigen::Matrix3d M;
  M << 1, -u(2), u(1), 
      u(2), 1, -u(0),
      -u(1), u(0), 1;

  closest_rotation(M, R);
  t = u.tail(3);
}

