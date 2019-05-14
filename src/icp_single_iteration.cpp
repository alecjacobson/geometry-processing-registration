#include "icp_single_iteration.h"
#include "point_to_plane_rigid_matching.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "random_points_on_mesh.h"
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
  Eigen::MatrixXd X;
  Eigen::VectorXd D;
  Eigen::MatrixXd P; 
  Eigen::MatrixXd N;


  std::cout << "1" << std::endl;

  random_points_on_mesh(num_samples, VX, FX, X);
  std::cout << "2" << std::endl;
  point_mesh_distance(X, VY, FY, D, P, N);
  std::cout << "3" << std::endl;
  if (method == ICP_METHOD_POINT_TO_POINT){
    std::cout << "4" << std::endl;
    point_to_point_rigid_matching(X, P, R, t);
  } 
  else{
    std::cout << "5" << std::endl;
    point_to_plane_rigid_matching(X, P, N, R, t);
  }
  std::cout << "6" << std::endl;
}
