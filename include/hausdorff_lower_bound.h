#ifndef HAUSDORFF_LOWER_BOUND_H
#define HAUSDORFF_LOWER_BOUND_H
#include <Eigen/Core>
#include "point_mesh_distance.h"
#include "random_points_on_mesh.h"
#include "hausdorff_lower_bound.h"
// Compute a lower bound on the _directed_ Hausdorff distance from a given mesh
// (VX,FX) to another mesh (VY,FY).
//
//
// Inputs:
//   VX  #VX by 3 list of mesh vertex positions
//   FX  #FX by 3 list of triangle mesh indices into VX
//   VY  #VY by 3 list of mesh vertex positions
//   FY  #FY by 3 list of triangle mesh indices into VY
//   n  number of random samples to use (larger --> more accurate)
// Returns lower bound 
double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n);
#endif
