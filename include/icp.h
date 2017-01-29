#ifndef ICP_H
#define ICP_H
#include <Eigen/Core>

enum ICPMethod
{
  ICP_METHOD_POINT_TO_POINT = 0,
  ICP_METHOD_POINT_TO_PLANE = 1,
  NUM_ICP_METHODS = 2,
};
// Using the iterative closest point method align (`VX`,`FX`) to (`VY`,`FY`) by
// finding the rigid transformation (`R`,`t`)minimizing the matching energy:
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
//   num_iters  maximum number of iterations to run
//   method  method of linearization to use, one ot ICP_METHOD_POINT_TO_POINT
//     or ICP_METHOD_POINT_TO_PLANE
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
//   X  num_samples by 3 list of positions of sample points on (VX,FX)
//   P  num_samples by 3 list of closest points on (VY,FY) to points in X
void icp(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const int num_iters,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t,
  Eigen::MatrixXd & X,
  Eigen::MatrixXd & P);

#endif
