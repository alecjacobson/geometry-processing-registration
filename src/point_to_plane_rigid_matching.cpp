#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_plane_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	const Eigen::MatrixXd & N,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	// compute [diag(N0), diag(N1), diag(N2)]
	Eigen::MatrixXd normal(X.rows(), 3*X.rows());
	Eigen::MatrixXd N0 = N.col(0).asDiagonal();
	Eigen::MatrixXd N1 = N.col(1).asDiagonal();
	Eigen::MatrixXd N2 = N.col(2).asDiagonal();
	normal << N0, N1, N2;
	// compute A
	Eigen::RowVectorXd zero = Eigen::RowVectorXd::Zero(X.rows());
	Eigen::RowVectorXd ones = Eigen::RowVectorXd::Ones(X.rows());
	Eigen::MatrixXd A(3*X.rows(), 6);
	A << zero, X.col(2), -X.col(1), ones, zero, zero,
		-X.col(2), 0, X.col(0), zero, ones, zero,
		X.col(1), -X.col(0), zero, zero, zero, ones;
	// compute X - P
	Eigen::MatrixXd xp(3*X.rows(), 3);
	xp << X.col(0) - P.col(0), X.col(1) - P.col(1), X.col(2) - P.col(2);
	// compute u
	A = normal * A;
	xp = normal * xp;
	Eigen::RowVectorXd u;
	// u = [alpha, beta, gamma, t0, t1, t2]
	u = (A.transpose() * A).inverse() * -A.transpose() * xp;
	// get t
	t << u(3), u(4), u(5);
	// compute M and R
	Eigen::Matrix3d M;
	M << 1, -u(2), u(1), u(2), 1, -u(0), -u(1), u(0), 1;
	closest_rotation(M, R);
}
