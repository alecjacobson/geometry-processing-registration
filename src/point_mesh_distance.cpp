#include "point_mesh_distance.h"

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  D = (X-P).rowwise().norm();
}
