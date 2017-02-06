#include "closest_rotation.h"

#include <Eigen/Dense>
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
	// Replace with your code
	// Reference:	https://igl.ethz.ch/projects/ARAP/svd_rot.pdf

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);	
	
	Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
	sigma.diagonal() << 1, 1, (svd.matrixU() * svd.matrixV().transpose()).determinant();
	
	R = svd.matrixV() * sigma * (svd.matrixU().transpose());
}
