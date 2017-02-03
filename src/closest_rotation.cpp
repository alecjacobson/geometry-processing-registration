#include "closest_rotation.h"
#include <Eigen/SVD>

using namespace Eigen;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  //// Replace with your code
  //R = Eigen::Matrix3d::Identity();
	JacobiSVD<Matrix3d> svd(M);

	Vector3d lambdaDiag(1, 1, (svd.matrixV()*svd.matrixU().transpose()).determinant());

	R = svd.matrixV()*lambdaDiag.asDiagonal()*(svd.matrixU().transpose());
}
