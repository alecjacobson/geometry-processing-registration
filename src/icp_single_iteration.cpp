#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"

using namespace Eigen;

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
  VectorXd D;
  MatrixXd points, P, N;
  random_points_on_mesh(num_samples, VX, FX, points);
  point_mesh_distance(points, VY, FY, D, P, N);
  if (method == ICP_METHOD_POINT_TO_POINT) {
    point_to_point_rigid_matching(points, P, R, t);
  }
  else{
    point_to_plane_rigid_matching(points, P, N, R, t);
  }
}


