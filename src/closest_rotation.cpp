#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
	using namespace Eigen;
	
	// Calculate UV
	JacobiSVD<Matrix3d> svd(M, ComputeThinU | ComputeThinV);
	Matrix3d U, V, omega;
	U = svd.matrixU();
	V = svd.matrixV();
	
	// Calculate omega
	omega = Matrix3d::Identity();
	omega(2, 2) = (U * V.transpose()).determinant();
	
	// Calculate R
	R.resize(3, 3);
	R = U * omega * V.transpose();
}
