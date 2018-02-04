#ifndef ICP_SINGLE_ITERATION_H
#define ICP_SINGLE_ITERATION_H
#include <Eigen/Core>
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"

enum ICPMethod
{
  ICP_METHOD_POINT_TO_POINT = 0,
  ICP_METHOD_POINT_TO_PLANE = 1,
  NUM_ICP_METHODS = 2,
};
// Conduct a single iteration of the iterative closest point method align
// (`VX`,`FX`) to (`VY`,`FY`) by finding the rigid transformation
// (`R`,`t`)minimizing the matching energy:
//
// \\[
//    ∫_X ‖ Rx+t - P_Y( Rx +t ) ‖² dA
// \\]
//
//
// Inputs:
//   VX  #VX by 3 list of mesh vertex positions
//   FX  #FX by 3 list of triangle mesh indices into VX
//   VY  #VY by 3 list of mesh vertex positions
//   FY  #FY by 3 list of triangle mesh indices into VY
//   num_samples  number of random samples to use (larger --> more accurate)
//   method  method of linearization to use, one ot ICP_METHOD_POINT_TO_POINT
//     or ICP_METHOD_POINT_TO_PLANE
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t);

#endif
