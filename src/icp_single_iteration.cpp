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
  Eigen::MatrixXd X, P, N;
  random_points_on_mesh(num_samples, VX, FX, X);
  Eigen::VectorXd D;
  point_mesh_distance(X, VY, FY, D, P, N);
  if (method == ICP_METHOD_POINT_TO_PLANE) {
    // To Plane
    point_to_plane_rigid_matching(X, P, N, R, t);
    return;
  }
  
  // To Point
  point_to_point_rigid_matching(X, P, R, t);
}
