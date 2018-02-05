#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{

  Eigen::MatrixXd X_samp;
  Eigen::VectorXd min_distances;
  Eigen::MatrixXd closest_points;
  Eigen::MatrixXd normals;

  random_points_on_mesh(num_samples, VX, FX, X_samp);
  point_mesh_distance(X_samp, VY, FY, min_distances, closest_points, normals);

  point_to_point_rigid_matching(X_samp, closest_points, R, t);
}
