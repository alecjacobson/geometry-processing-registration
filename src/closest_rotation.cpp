#include "closest_rotation.h"
#include <Eigen/SVD>
#include<Eigen/LU>

using namespace Eigen;

void closest_rotation(
		const Eigen::Matrix3d& M,
		Eigen::Matrix3d& R) {

	// Compute U and V for the decomposition.
	JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
	const Matrix3d& u = svd.matrixU();
	const Matrix3d& v = svd.matrixV();
	Matrix3d vT = svd.matrixV();
	vT.transposeInPlace();

	Eigen::Matrix3d omega = Matrix3d::Identity();
	double det = (u * vT).determinant();
	omega(2, 2) = det;

	R = u * omega * vT;
}
