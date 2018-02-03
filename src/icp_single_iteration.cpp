#include "icp_single_iteration.h"
#include <stdexcept>
#include <exception>
#include "point_to_point_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
/***********************************************************************
Conduct a single iteration of the iterative closest point method
align (VX,FX) to (VY,FY) by finding the rigid transformation (R,t)
minimizing the matching energy.
************************************************************************
The caller can specify the number of samples num_samples used to
approximate the integral over XX and specify the method (point-to-point
or point-to-plane).
*************************************************************************/
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
  using namespace Eigen;
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd  X;
  random_points_on_mesh(num_samples, VX, FX, X);

  Eigen::VectorXd D;
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(X, VY, FY, D, P, N);

  if (method == ICP_METHOD_POINT_TO_POINT)
    point_to_point_rigid_matching(X, P, R, t);
  else if (method == ICP_METHOD_POINT_TO_PLANE)
    point_to_plane_rigid_matching(X, P, N, R, t);
}
