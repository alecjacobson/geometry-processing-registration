#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
#include <stdio.h>

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
	MatrixXd X;
	random_points_on_mesh(num_samples, VX, FX, X);

	VectorXd D;
	MatrixXd P, N;
	point_mesh_distance(X, VY, FY, D, P, N);
	
	switch (method)
	{
		case ICP_METHOD_POINT_TO_POINT:
		{
			point_to_point_rigid_matching(X, P, R, t);
			break;
		}
		case ICP_METHOD_POINT_TO_PLANE:
		{
			point_to_plane_rigid_matching(X, P, N, R, t);
			break;
		}
	}
}
