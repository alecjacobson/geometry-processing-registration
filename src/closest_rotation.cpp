#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

using namespace Eigen;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
	JacobiSVD<Matrix3d> svd(M, ComputeFullU | ComputeFullV);

	//assuming Alec's formulation has two typos
	Vector3d lambdaDiag(1, 1, (svd.matrixV()*svd.matrixU().transpose()).determinant());

	R = svd.matrixV()*lambdaDiag.asDiagonal()*(svd.matrixU().transpose());

	////assuming Alec's formulation has one typo
	//Vector3d lambdaDiag(1, 1, (svd.matrixU()*svd.matrixV().transpose()).determinant());

	//R = svd.matrixU()*lambdaDiag.asDiagonal()*(svd.matrixV().transpose());
}
