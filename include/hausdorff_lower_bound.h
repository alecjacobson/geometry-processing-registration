#ifndef HAUSDORFF_LOWER_BOUND_H
#define HAUSDORFF_LOWER_BOUND_H
#include <Eigen/Core>
// Compute a lower bound on the _directed_ Hausdorff distance from a given mesh
// (VX,FX) to another mesh (VY,FY).
//
//
// Inputs:
//   n  number of random samples to use (larger --> more accurate)
//   VX  #VX by 3 list of mesh vertex positions
//   FX  #FX by 3 list of triangle mesh indices into VX
//   VY  #VY by 3 list of mesh vertex positions
//   FY  #FY by 3 list of triangle mesh indices into VY
// Returns lower bound 
double hausdorff_lower_bound(
  const int n,
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY);
#endif
