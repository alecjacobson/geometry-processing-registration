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
	Eigen::MatrixXd X(n, 3);
	random_points_on_mesh(n, VX, FX, X);
	Eigen::MatrixXd P, N;
	Eigen::VectorXd D;
	point_mesh_distance(X, VY, FY, D, P, N);
	return D.maxCoeff();
}
