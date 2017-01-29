#ifndef RANDOM_POINTS_ON_MESH_H
#define RANDOM_POINTS_ON_MESH_H
#include <Eigen/Core>
// RANDOM_POINTS_ON_MESH Randomly sample a mesh (V,F) n times.
//
// Inputs:
//   n  number of samples
//   V  #V by dim list of mesh vertex positions
//   F  #F by 3 list of mesh triangle indices
// Outputs:
//   B  n by 3 list of barycentric coordinates, ith row are coordinates of
//     ith sampled point in face FI(i)
//   FI  n list of indices into F 
//
void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & B,
  Eigen::VectorXi & FI);
#endif
