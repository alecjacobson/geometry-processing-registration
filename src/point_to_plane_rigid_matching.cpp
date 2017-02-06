#include "point_to_plane_rigid_matching.h"
#include <closest_rotation.h>
#include <igl/polar_svd.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;

void point_to_plane_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	const Eigen::MatrixXd & N,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{	
	int k = X.rows();
	auto X_1 = X.col(0);
	auto X_2 = X.col(1);
	auto X_3 = X.col(2);

	MatrixXd I(k, 1);
	I.setOnes();

	MatrixXd A(3 * k, 6);
	A.setZero();
	A.block(0, 1, k, 1) = X_3;
	A.block(0, 2, k, 1) = -X_2;
	A.block(0, 3, k, 1) = I;

	A.block(k, 0, k, 1) = -X_3;
	A.block(k, 2, k, 1) = X_1;
	A.block(k, 4, k, 1) = I;

	A.block(2 * k, 0, k, 1) = X_2;
	A.block(2 * k, 1, k, 1) = -X_1;
	A.block(2 * k, 5, k, 1) = I;

	MatrixXd XP(3 * k, 1);
	XP.block(0, 0, k, 1) = X_1 - P.col(0);
	XP.block(k, 0, k, 1) = X_2 - P.col(1);
	XP.block(2 * k, 0, k, 1) = X_3 - P.col(2);
	
	MatrixXd diag_N(k, 3 * k);
	diag_N.setZero();
	diag_N.block(0, 0, k, k) = N.col(0).asDiagonal();
	diag_N.block(0, k, k, k) = N.col(1).asDiagonal();
	diag_N.block(0, 2 * k, k, k) = N.col(2).asDiagonal();
		
	MatrixXd left = A.transpose()*diag_N.transpose()*diag_N*A;
	MatrixXd right = -1*A.transpose()*diag_N.transpose()*diag_N*XP;
	VectorXd u = left.colPivHouseholderQr().solve(right);

	auto alpha = u(0), beta = u(1), gamma = u(2);
	t << u(3), u(4), u(5);

	Matrix3d M;
	M << 1, -gamma, beta,
		gamma, 1, -alpha,
		-beta, alpha, 1;
	
	closest_rotation(M, R);
}
