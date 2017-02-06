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
	
	//Sample points to approximate hausdorrf distance
	Eigen::MatrixXd X;
	random_points_on_mesh(n, VX, FX, X);

	//Find minimum distances from points on sampled mesh to other mesh
	Eigen::VectorXd D;
	Eigen::MatrixXd P, N;
	point_mesh_distance(X, VY, FY, D, P, N);

	//Return the maximum distance
	return D.maxCoeff();	
}
