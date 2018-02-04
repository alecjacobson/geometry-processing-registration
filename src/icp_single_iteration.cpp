#include "icp_single_iteration.h"

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
  // Get a random sample X
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);
  // Get Y_o and N
  Eigen::MatrixXd Y_o;
  Eigen::VectorXd D;
  Eigen::MatrixXd N;
  point_mesh_distance(X, VY, FY, D, Y_o, N);

  if(method == ICPMethod::ICP_METHOD_POINT_TO_POINT){
    point_to_point_rigid_matching(X, Y_o, R, t);
  } else{
    point_to_plane_rigid_matching(X, Y_o, N, R, t);
  }
}
