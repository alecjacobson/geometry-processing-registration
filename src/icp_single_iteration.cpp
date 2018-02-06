#include "icp_single_iteration.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"
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
  /*
    X ← sample source mesh (V_X,F_X)
    P0 ← project all X onto target mesh (V_Y,F_Y)
    R,t ← update rigid transform to best match X and P0
    V_X ← rigidly transform original source mesh by R and t
  */
  Eigen::MatrixXd X;
  Eigen::MatrixXd P;
  Eigen::VectorXd D;
  Eigen::MatrixXd N;

  std::cout << "1" <<std::endl;
  random_points_on_mesh(num_samples, VX, FX, X);
  std::cout << "2" <<std::endl;
  point_mesh_distance(X, VY, FY, D, P, N);
  std::cout << "3" <<std::endl;
  if (method == ICP_METHOD_POINT_TO_POINT){
    point_to_point_rigid_matching(X, P, R, t);
  } else if (method == ICP_METHOD_POINT_TO_PLANE){
    point_to_plane_rigid_matching(X, P, N, R, t);
  }
  std::cout << "4" <<std::endl;
}
