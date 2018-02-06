#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include <iostream>

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
  std::cout << "test1" << std::endl;
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);
  std::cout << "test2" << std::endl;
  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(X, VY, FY, D, P, N);

  std::cout << "test3" << std::endl;
  switch(method)
  {
    case ICP_METHOD_POINT_TO_POINT: point_to_point_rigid_matching(X, P, R, t);   break;
    case ICP_METHOD_POINT_TO_PLANE: point_to_plane_rigid_matching(X, P, N, R, t); break;
  }
}
