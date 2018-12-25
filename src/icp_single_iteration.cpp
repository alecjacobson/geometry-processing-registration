#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
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
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  // Sample points from X
  Eigen::MatrixXd sampled_X(num_samples, 3);

  random_points_on_mesh(
  num_samples,
  VX,
  FX,
  sampled_X);

  Eigen::VectorXd D;
  Eigen::MatrixXd P, N;

  point_mesh_distance(
  sampled_X,
  VY,
  FY,
  D,
  P,
  N);


  if (method == ICP_METHOD_POINT_TO_POINT) {

    point_to_point_rigid_matching(
    sampled_X,
    P,
    R,
    t);

  } else if (method == ICP_METHOD_POINT_TO_PLANE) {
    
    point_to_plane_rigid_matching(
    sampled_X,
    P,
    N,
    R,
    t);

  }
}
