#ifndef POINT_TO_PLANE_RIGID_MATCHING_H
#define POINT_TO_PLANE_RIGID_MATCHING_H
#include <Eigen/Core>
#include <closest_rotation.h>

// Given a set of source points `X` and corresponding target points `P` and
// normals `N`, find the optimal rigid transformation (`R`,`t`) that aligns `X`
// to `P`, minimizing the matching energy:
//
//   (R X' - t' 1' - P')²
//
// Inputs:
//   X  #X by 3 set of source points
//   P  #X by 3 set of target points
//   N  #X by 3 set of target normals
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
//   
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t);
#endif


