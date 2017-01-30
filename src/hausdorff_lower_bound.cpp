#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
double hausdorff_lower_bound(
	const Eigen::MatrixXd & VX,
	const Eigen::MatrixXi & FX,
	const Eigen::MatrixXd & VY,
	const Eigen::MatrixXi & FY,
	const int n)
{
	//Computing the lower bound of a directed hausdorff distance. 
	//First, randomly sample the X mesh.
	Eigen::MatrixXd samples;
	random_points_on_mesh(n, VX, FX, samples);
	//Find the distances to the mesh.

	Eigen::VectorXd D;
	Eigen::MatrixXd P, N;
	point_mesh_distance(samples, VY, FY, D, P, N);
	return D.maxCoeff();
}
