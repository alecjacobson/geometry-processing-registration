#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
	const Eigen::Matrix3d & M,
  	Eigen::Matrix3d & R)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXd U = svd.matrixU();
	Eigen::MatrixXd V = svd.matrixV();
	Eigen::Matrix3d Omega(3, 3);
	double det = (U * V.transpose()).determinant();
	Omega << 1, 0, 0, 0, 1, 0, 0, 0, det;
	R = U * Omega * V.transpose();
}
