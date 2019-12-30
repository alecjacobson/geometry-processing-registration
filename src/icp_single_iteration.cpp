#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"

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
  // Replace with your code

  // Sample source mesh
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);

  // Project sample points onto mesh Y
  Eigen::VectorXd D;
  Eigen::MatrixXd Py;
  Eigen::MatrixXd Ny;
  point_mesh_distance(X, VY, FY, D, Py, Ny);

  // Do one iteration of the chosen method to obtain an R and a t
  if (method == ICP_METHOD_POINT_TO_POINT) {
	point_to_point_rigid_matching(X, Py, R, t);
  }
  
  else if (method == ICP_METHOD_POINT_TO_PLANE) {
	point_to_plane_rigid_matching(X, Py, Ny, R, t);
  }
}
