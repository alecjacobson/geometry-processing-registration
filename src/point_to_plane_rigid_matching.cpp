#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>

#include "closest_rotation.h"

void point_to_plane_rigid_matching(const Eigen::MatrixXd & X,
		const Eigen::MatrixXd & P, const Eigen::MatrixXd & N,
		Eigen::Matrix3d & R, Eigen::RowVector3d & t) {
	// Replace with your code
	R = Eigen::Matrix3d::Identity();
	t = Eigen::RowVector3d::Zero();

	int row = X.rows();
	Eigen::VectorXd Z0 = Eigen::VectorXd::Zero(row);
	Eigen::VectorXd Z1 = Eigen::VectorXd::Ones(row);

	Eigen::VectorXd X0 = X.col(0);
	Eigen::VectorXd X1 = X.col(1);
	Eigen::VectorXd X2 = X.col(2);
	Eigen::MatrixXd A(3 * row, 6);
	A << Z0, X2, -X1, Z1, Z0, Z0, -X2, Z0, X0, Z0, Z1, Z0, X1, -X0, Z0, Z0, Z0, Z1;

	Eigen::MatrixXd b(3 * row, 1);
	b << P.col(0) - X0, P.col(1) - X1, P.col(2) - X2;

	/* faster
	 Eigen::MatrixXd diag(row, 3 * row);
	 diag << Eigen::MatrixXd(N.col(0).asDiagonal()), Eigen::MatrixXd(
	 N.col(1).asDiagonal()), Eigen::MatrixXd(N.col(2).asDiagonal());

	 A = diag * A;
	 b = diag * b;
	 */

	Eigen::MatrixXd ATA = A.transpose() * A;
	Eigen::MatrixXd ATb = A.transpose() * b;

	Eigen::VectorXd u = ATA.inverse() * ATb;
	Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
	double alpha = u[0];
	double beta = u[1];
	double gamma = u[2];

	M(1, 0) += -gamma;
	M(2, 0) += beta;
	M(0, 1) += gamma;
	M(2, 1) += -alpha;
	M(0, 2) += -beta;
	M(1, 2) += alpha;

	closest_rotation(M, R);

	t[0] = u[3];
	t[1] = u[4];
	t[2] = u[5];
}
