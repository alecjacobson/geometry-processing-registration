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
  Eigen::MatrixXd X_samp;
  Eigen::VectorXd min_distances;
  Eigen::MatrixXd closest_points;
  Eigen::MatrixXd normals;

  random_points_on_mesh(n, VX, FX, X_samp);
  point_mesh_distance(X_samp, VY, FY, min_distances, closest_points, normals);
  return min_distances.maxCoeff();
}
