#include "icp_single_iteration.h"

#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"

void icp_single_iteration(const Eigen::MatrixXd & VX,
		const Eigen::MatrixXi & FX, const Eigen::MatrixXd & VY,
		const Eigen::MatrixXi & FY, const int num_samples,
		const ICPMethod method, Eigen::Matrix3d & R, Eigen::RowVector3d & t) {
	// Replace with your code
	// R = Eigen::Matrix3d::Identity();
	// t = Eigen::RowVector3d::Zero();

	// first sample
	Eigen::MatrixXd Xsam;
	random_points_on_mesh(num_samples, VX, FX, Xsam);

	// close correspondence
	Eigen::VectorXd D;
	Eigen::MatrixXd P, N;
	point_mesh_distance(Xsam, VY, FY, D, P, N);

	// match
	// point_to_point_rigid_matching(Xsam, P, R, t);
	point_to_plane_rigid_matching(Xsam, P, N, R, t);
}

