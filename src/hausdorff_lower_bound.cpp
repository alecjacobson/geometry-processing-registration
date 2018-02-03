#include "hausdorff_lower_bound.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  // Sample X to be size n
  Eigen::MatrixXd X;
  random_points_on_mesh(n,VX,FX,X);
  // find the max of
  // for each point in sample X,
  // find the closest point in Y
  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(X, VY, FY, D, P, N);
  return D.maxCoeff();
}
