#include "hausdorff_lower_bound.h"
#include "point_mesh_distance.h"
#include "random_points_on_mesh.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  Eigen::MatrixXd points, N, P;
  Eigen::VectorXd D;
  random_points_on_mesh(n, VX, FX, points);
  point_mesh_distance(points, VY, FY, D, P, N);
  double lowerBound = D.maxCoeff();
  return lowerBound;
}


