#include "icp_single_iteration.h"
#include "point_triangle_distance.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
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
  //double d; Eigen::RowVector3d p;
  //Eigen::RowVector3d x(0, 0, 0);
  //point_triangle_distance(x, VX.row(0), VX.row(1), VX.row(2), d, p);
  // Replace with your code
  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);
  point_mesh_distance(X, VY, FY, D, P, N);
  if (method == ICP_METHOD_POINT_TO_POINT) {
	  point_to_point_rigid_matching(X, P, R, t);
  }
  else {
	  point_to_plane_rigid_matching(X, P, N, R, t);
  }
}
