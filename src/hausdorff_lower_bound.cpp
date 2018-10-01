#include "hausdorff_lower_bound.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  Eigen::MatrixXd pointsX, P, N;
  Eigen::VectorXd D;

  random_points_on_mesh(n, VX, FX, pointsX);
  point_mesh_distance(pointsX, VY, FY, D, P, N);

  return D.maxCoeff();
}