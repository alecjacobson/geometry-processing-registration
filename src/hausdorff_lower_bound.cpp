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
  // Replace with your code

  // randomly sample X
  Eigen::MatrixXd Px;
  random_points_on_mesh(n, VX, FX, Px);

  // compute projections of Px onto Y
  Eigen::VectorXd D;
  Eigen::MatrixXd P, N;
  point_mesh_distance(Px, VY, FY, D, P, N);

  // return max distance
  return D.maxCoeff();
}
