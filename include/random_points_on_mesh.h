#ifndef RANDOM_POINTS_ON_MESH_H
#define RANDOM_POINTS_ON_MESH_H
#include <Eigen/Core>
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <random>
// RANDOM_POINTS_ON_MESH Randomly sample a mesh (V,F) n times.
//
// Inputs:
//   n  number of samples
//   V  #V by dim list of mesh vertex positions
//   F  #F by 3 list of mesh triangle indices
// Outputs:
//   X  n by 3 list of random points on (V,F)
//
void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X);

int get_random_triangle(
  const Eigen::MatrixXd & C,
  const int n);

Eigen::RowVector3d get_random_point(
  const int triangle_idx,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F);

#endif
