#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(X.rows());
  Eigen::VectorXd zeros = Eigen::VectorXd::Zero(X.rows());

  Eigen::MatrixXd A(3 * X.rows(), 6);

  Eigen::VectorXd X_1 = X.col(0);
  Eigen::VectorXd X_2 = X.col(1);
  Eigen::VectorXd X_3 = X.col(2);

  Eigen::VectorXd P_1 = P.col(0);
  Eigen::VectorXd P_2 = P.col(1);
  Eigen::VectorXd P_3 = P.col(2);

  Eigen::VectorXd P_col(P_1.rows()*3);
  P_col << P_1, P_2, P_3;
  Eigen::VectorXd X_col(X_1.rows()*3);
  X_col << X_1, X_2, X_3;

  Eigen::VectorXd D = X_col - P_col;

  A.col(0) << zeros, -X_3, X_2;
  A.col(1) << X_3, zeros, -X_1;
  A.col(2) << -X_2, X_1, zeros;
  A.col(3) << ones, zeros, zeros;
  A.col(4) << zeros, ones, zeros;
  A.col(5) << zeros, zeros, ones;

  Eigen::VectorXd u = (A.transpose()*A).inverse() * (-A.transpose() * D);

  double alpha = u(0);
  double beta = u(1);
  double gamma = u(2);
  Eigen::Matrix3d r;
  r << 0, -gamma, beta,
	  gamma, 0, -alpha,
	  -beta, alpha, 0;
  R = Eigen::Matrix3d::Identity() + r.transpose();
  t = Eigen::RowVector3d(u(3),u(4),u(5));
}

