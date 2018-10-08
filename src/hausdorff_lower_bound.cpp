#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include <iostream>

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  Eigen::MatrixXd X, P, N;
  Eigen::VectorXd D;
  random_points_on_mesh(n, VX, FX, X);
  std::cout << "random sampling compelted" << std::endl;
  point_mesh_distance(X, VY, FY, D, P, N);
  std::cout << "distances calculated" << std::endl;
  return D.maxCoeff();
}
