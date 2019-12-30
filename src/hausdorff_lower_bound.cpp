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
  // Sample points from X mesh
  Eigen::MatrixXd sample_P;
  random_points_on_mesh(n, VX, FX, sample_P);

  // Figure out their closest points to the Y mesh
  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(sample_P, VY, FY, D, P, N);

  // compute max D for the hausdorff lb
  return D.maxCoeff();
}
