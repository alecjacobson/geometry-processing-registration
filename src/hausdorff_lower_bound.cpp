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
  Eigen::MatrixXd sample;
  random_points_on_mesh(n, VX, FX, sample);
  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(sample, VY, FY, D, P, N);
  return D.maxCoeff();
}
