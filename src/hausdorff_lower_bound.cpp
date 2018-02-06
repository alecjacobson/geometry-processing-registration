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
  // Replace with your code
	// Random sample points on mesh VX
	Eigen::MatrixXd X;  // Points randomly sampled on X mesh
	random_points_on_mesh(n, VX, FX, X);

	// Compute the shortest point to mesh distance for all sampled points
	Eigen::VectorXd D;
	Eigen::MatrixXd P;
	Eigen::MatrixXd N;
	point_mesh_distance(X, VY, FY, D, P, N);
	
	// Get the hausdorff lower bound
	return D.maxCoeff();
}
