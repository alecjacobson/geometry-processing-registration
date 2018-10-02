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
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd pointsX, P, N;
  Eigen::VectorXd D;

  random_points_on_mesh(num_samples, VX, FX, pointsX);
  point_mesh_distance(pointsX, VY, FY, D, P, N);

  if(method == ICP_METHOD_POINT_TO_POINT){
    point_to_point_rigid_matching(pointsX, P, R, t);
  }
  else{
    point_to_plane_rigid_matching(pointsX, P, N, R, t);
  }
}
