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
  Eigen::MatrixXd X;
  Eigen::MatrixXd Y;
  random_points_on_mesh(n, VX, FX, X);
  random_points_on_mesh(n, VY, FY, Y);

  Eigen::VectorXd DX;
  Eigen::MatrixXd PX;
  Eigen::MatrixXd NX;
  point_mesh_distance(X, VX, FX, DX, PX, NX);

  Eigen::VectorXd DY;
  Eigen::MatrixXd PY;
  Eigen::MatrixXd NY;
  point_mesh_distance(Y, VY, FY, DY, PY, NY);

  double minumum = std::min(DX.minCoeff(), DY.minCoeff());
  return std::floor(minumum);
}
