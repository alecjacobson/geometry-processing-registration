#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"

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
  // Sample points on X
  Eigen::MatrixXd X(num_samples, 3);
  random_points_on_mesh(num_samples, VX, FX, X);
  
  // Find closest points on Y
  Eigen::VectorXd D(num_samples);
  Eigen::MatrixXd P(num_samples, 3), N(num_samples, 3);
  point_mesh_distance(X, VY, FY, D, P, N);
  
  // Find optimal R & t
  if (method == ICP_METHOD_POINT_TO_POINT) {
    point_to_point_rigid_matching(X, P, R, t);
  } else {
    point_to_plane_rigid_matching(X, P, N, R, t);
  }
}
