#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

using namespace Eigen;

double hausdorff_lower_bound(
	const Eigen::MatrixXd & VX,
	const Eigen::MatrixXi & FX,
	const Eigen::MatrixXd & VY,
	const Eigen::MatrixXi & FY,
	const int n)
{
	MatrixXd X;
	random_points_on_mesh(n, VX, FX, X);

	VectorXd D;
	MatrixXd P, N;
	point_mesh_distance(X, VY, FY, D, P, N);

	return D.maxCoeff();
}
