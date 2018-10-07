#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code

  // Sample X
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);

  // Compute P
  Eigen::MatrixXd P, N;
  Eigen::VectorXd D;
  point_mesh_distance(X, VY, FY, D, P, N);

  point_to_point_rigid_matching(X, P, R, t);
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();
}
