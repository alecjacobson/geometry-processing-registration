#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
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
	/*
	icp V_X, F_X, V_Y, F_Y
		R, t <- initialize(e.g., set to identity transformation)
		repeat until convergence
		X <- sample source mesh(V_X, F_X)
		P0 <- project all X onto target mesh(V_Y, F_Y)
		R, t <- update rigid transform to best match X and P0
		V_X <- rigidly transform original source mesh by R and t
	*/

  //R = Eigen::Matrix3d::Identity();
  //t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples, VX, FX, X);
  std::cout << "random points on mesh" << std::endl;
  Eigen::MatrixXd P; 
  Eigen::MatrixXd N;
  Eigen::VectorXd D;
  point_mesh_distance(X, VY, FY, D, P, N);
  std::cout << "distance to mesh" << std::endl;
  Eigen::Matrix3d M;
  point_to_plane_rigid_matching(X, P, N, M, t);
  //point_to_point_rigid_matching(X, P, M, t);
  std::cout << "rigid matching" << std::endl;
  closest_rotation(M,R);
  std::cout << "closest rotation" << std::endl;
  std::cout << "---------" << std::endl;

}
