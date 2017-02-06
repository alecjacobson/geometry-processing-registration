#include "closest_rotation.h"
#include "igl/svd3x3.h"

using namespace Eigen;

void closest_rotation(
	const Eigen::Matrix3d & M,
	Eigen::Matrix3d & R)
{
	Matrix3d U, V;
	Matrix<double, 3, 1> S;
	igl::svd3x3(M, U, S, V);

	Matrix3d G;
	G.setIdentity();
	G(2, 2) = (V*U.transpose()).determinant();

	R = V*G*U.transpose();
}
