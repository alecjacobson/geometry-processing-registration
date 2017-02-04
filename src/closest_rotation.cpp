#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullV | Eigen::ComputeFullU);
	svd.computeU();
	Eigen::Matrix3d U = svd.matrixU();
	svd.computeV();
	Eigen::Matrix3d V = svd.matrixV();

	Eigen::Matrix3d Omega = Eigen::Matrix3d::Zero();
	Omega(0, 0) = 1;
	Omega(1, 1) = 1;
	Omega(2, 2) = (U*V.transpose()).determinant();

	R = U*Omega*V.transpose();
}
