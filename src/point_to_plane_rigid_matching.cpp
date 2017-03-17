#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

using namespace Eigen;

void point_to_plane_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	const Eigen::MatrixXd & N,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	int k = X.rows();

	MatrixXd A(3 * k, 6), Ndiag(k, 3 * k);
	VectorXd pMinusX(3 * k);
	pMinusX.head(k) = P.col(0) - X.col(0);
	pMinusX.segment(k, k) = P.col(1) - X.col(1);
	pMinusX.tail(k) = P.col(2) - X.col(2);

	A.setZero();

	A.block(0, 1, k, 1) = X.col(2);
	A.block(0, 2, k, 1) = -X.col(1);
	A.block(k, 0, k, 1) = -X.col(2);
	A.block(k, 2, k, 1) = X.col(0);
	A.block(2 * k, 0, k, 1) = X.col(1);
	A.block(2 * k, 1, k, 1) = -X.col(0);

	A.block(0, 3, k, 1) = VectorXd::Ones(k);
	A.block(k, 4, k, 1) = VectorXd::Ones(k);
	A.block(2 * k, 5, k, 1) = VectorXd::Ones(k);

	Ndiag.setZero();

	Ndiag.block(0, 0, k, k) = N.col(0).asDiagonal();
	Ndiag.block(0, k, k, k) = N.col(1).asDiagonal();
	Ndiag.block(0, 2 * k, k, k) = N.col(2).asDiagonal();

	MatrixXd coeffMat = A.transpose()*Ndiag.transpose()*Ndiag*A;
	MatrixXd constMat = A.transpose()*Ndiag.transpose()*Ndiag*(pMinusX);

	VectorXd u = coeffMat.jacobiSvd(ComputeThinU | ComputeThinV).solve(constMat);

	t = u.tail<3>().transpose();

	Matrix3d M = Matrix3d::Identity();
	M(0, 1) += -u(2);
	M(1, 0) += u(2);
	M(0, 2) += u(1);
	M(2, 0) += -u(1);
	M(1, 2) += -u(0);
	M(2, 1) += u(0);

	closest_rotation(M, R);
}
