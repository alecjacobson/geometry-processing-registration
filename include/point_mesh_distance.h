#ifndef POINT_MESH_DISTANCE_H
#define POINT_MESH_DISTANCE_H
#include <Eigen/Core>
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"
#include <iostream>
// Compute the distances `D` between a set of given points `X` and their
// closest points `P` on a given mesh with vertex positions `VY` and face
// indices `FY`.
//
// Inputs:
//   X  #X by 3 list of query positions
//   VY  #VY by 3 list of mesh vertex positions
//   FY  #FY by 3 list of triangle mesh indices into VY
// Outputs:
//   D  #X list of distances from X to P 
//   P  #X by 3 list of 3d position of closest points
//   N  #X by 3 list of 3d unit normal vectors of closest points
void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N);

#endif

