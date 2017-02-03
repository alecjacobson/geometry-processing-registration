#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"

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
	MatrixXd PX(num_samples, 3);
	random_points_on_mesh(num_samples, VX, FX, PX);

	VectorXd D0;
	MatrixXd P0, N0;

	point_mesh_distance(PX, VY, FY, D0, P0, N0);

	if (method == ICP_METHOD_POINT_TO_POINT)
		point_to_point_rigid_matching(PX, P0, R, t);
	else
		point_to_plane_rigid_matching(PX, P0, N0, R, t);
}
