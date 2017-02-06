#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include <closest_rotation.h>
#include <Eigen/Dense>

using namespace Eigen;

void point_to_point_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	int method = 1;

	if (method == 0) 
	{
		/*
		RowVector3d x_centroid = X.colwise().sum() / X.rows();
		RowVector3d p_centroid = P.colwise().sum() / P.rows();

		MatrixXd x_centers = x_centroid.replicate(X.rows(), 1);
		MatrixXd p_centers = p_centroid.replicate(P.rows(), 1);

		auto x_bar = X - x_centers;
		auto p_bar = P - p_centers;

		auto M = x_bar.transpose()*p_bar;

		closest_rotation(M, R);
		R = R.inverse()
		t = p_centroid.transpose() - R*x_centroid.transpose();
		*/
	}
	else
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

		MatrixXd left = A.transpose()*A;
		MatrixXd right = -1 * A.transpose()*XP;
		VectorXd u = left.colPivHouseholderQr().solve(right);

		auto alpha = u(0), beta = u(1), gamma = u(2);
		t << u(3), u(4), u(5);

		Matrix3d M;
		M << 1, -gamma, beta,
			gamma, 1, -alpha,
			-beta, alpha, 1;
		closest_rotation(M, R);
	}
}

