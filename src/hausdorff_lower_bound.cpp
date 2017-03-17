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
	MatrixXd PX(n, 3);
	random_points_on_mesh(n, VX, FX, PX);

	MatrixXd PY, N;
	VectorXd D;

	point_mesh_distance(PX, VY, FY, D, PY, N);

	return D.maxCoeff();
}
