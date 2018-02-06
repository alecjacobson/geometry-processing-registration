#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include <iostream>

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
	// Get the random sampling on X
	Eigen::MatrixXd X;
	random_points_on_mesh(n, VX, FX, X);

	// Compute the set of closest points for all X samples
	Eigen::VectorXd D;
	Eigen::MatrixXd P, N;
	point_mesh_distance(X, VY, FY, D, P, N);

	// Now find the maximum of these distances. This is the required bound.
	int i;
	double H = D.maxCoeff(&i);
	// std::cout << H << std::endl;

	return H;

}
