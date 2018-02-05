#ifndef VECT_TO_SKEW_H
#define VECT_TO_SKEW_H
#include <Eigen/Core>
// VECT_TO_SKEW_H Turn vector into skew symmetric matrix.
//
// Inputs:
//   v  vector (3D)
// Outputs:
//   X  matrix (3x3)
//
Eigen::Matrix3d vect_to_skew(const Eigen::VectorXd v);
#endif
