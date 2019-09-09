#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

using namespace Eigen;

void icp_single_iteration(
		const Eigen::MatrixXd & VX,
		const Eigen::MatrixXi & FX,
		const Eigen::MatrixXd & VY,
		const Eigen::MatrixXi & FY,
		const int num_samples,
		const ICPMethod method,
		Eigen::Matrix3d & R,
		Eigen::RowVector3d & t) {

	MatrixXd X;
	random_points_on_mesh(num_samples, VX, FX, X);

	VectorXd distances;
	MatrixXd points;
	MatrixXd normals;
	point_mesh_distance(X, VY, FY, distances, points, normals);

	if (method == ICPMethod::ICP_METHOD_POINT_TO_PLANE) {
		point_to_plane_rigid_matching(X, points, normals, R, t);
	} else {
		point_to_point_rigid_matching(X, points, R, t);
	}
}
