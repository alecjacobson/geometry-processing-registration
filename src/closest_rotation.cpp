#include "closest_rotation.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#define ESP 1E-3
using namespace Eigen;
// Given a 3¡Á3 matrix `M`, find the closest rotation matrix `R`.
//
// Inputs:
//   M  3x3 matrix
// Outputs:
//   R  3x3 rotation matrix
//
//we should project M onto the space of rotation matrices
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  JacobiSVD<Matrix3d> svd(M, DecompositionOptions::ComputeThinU | DecompositionOptions::ComputeThinV);
  Matrix3d U = svd.matrixU();
  Matrix3d V = svd.matrixV();
  Matrix3d Omega = Matrix3d::Identity();
  double UV_det = (U*V.transpose()).determinant();
  if (abs(UV_det+1)<ESP)
    Omega(2, 0) = -1;
  Omega(2, 2) = UV_det;
  R = U*Omega*V.transpose();
}
