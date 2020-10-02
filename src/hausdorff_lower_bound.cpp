#include "hausdorff_lower_bound.h"
#include "point_mesh_distance.h"
#include "random_points_on_mesh.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  // Replace with your code
  // return 0;

  // My code
	Eigen::MatrixXd X, P, N;
	Eigen::VectorXd D;
	random_points_on_mesh(n, VX, FX, X);
	point_mesh_distance(X, VY, FY, D, P, X); 
	return D.maxCoeff();
}
