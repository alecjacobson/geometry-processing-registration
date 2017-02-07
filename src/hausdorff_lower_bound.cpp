#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  // the directed hausdorff distance comes from the max of the distances between points in X and Y
  // randomly sampling with n samples from the X mesh:
  Eigen::MatrixXd X(n,3);
  random_points_on_mesh(n, VX, FX, X);

  // now get the distances between the sampled points and the set Y:
  Eigen::VectorXd D;
  Eigen::MatrixXd P, N;
  point_mesh_distance(X, VY, FY, D, P, N);

  // take the max of these points:
  return D.maxCoeff();
}
