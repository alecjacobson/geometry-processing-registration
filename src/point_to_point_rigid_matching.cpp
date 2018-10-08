#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	// closed form
	Eigen::RowVector3d x = X.colwise().mean();
	Eigen::RowVector3d p = P.colwise().mean();
	Eigen::MatrixXd x_bar = X.rowwise() - x;
	Eigen::MatrixXd p_bar = P.rowwise() - p;

	Eigen::Matrix3d M = p_bar.transpose() * x_bar;
	closest_rotation(M, R);

	// p - (R * x^T)^T = p - x * R^T
	t = p - x * R.transpose();
}

