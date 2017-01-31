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
  // Sample points on X
  Eigen::MatrixXd X(n, 3);
  random_points_on_mesh(n, VX, FX, X);
  
  // Find distances
  Eigen::VectorXd D(n);
  Eigen::MatrixXd P(n, 3), N(n, 3);
  point_mesh_distance(X, VY, FY, D, P, N);
  
  return D.maxCoeff();
}
