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
  // Replace with your code
	int k = X.rows();
	// Approximate point-to-plane minimizer
	// Construct diag_N
	Eigen::MatrixXd diag_N;
	diag_N.resize(k, 3*k);
	diag_N.block(0, 0, k, k) = N.col(0).asDiagonal();
	diag_N.block(0, k, k, k) = N.col(1).asDiagonal();
	diag_N.block(0, 2*k, k, k) = N.col(2).asDiagonal();

	// Construct A
	Eigen::MatrixXd A;
	A = Eigen::MatrixXd::Zero(k*3, 6);
	A.block(0, 1, k, 1) = X.col(2);
	A.block(0, 2, k, 1) = -X.col(1);
	A.block(0, 3, k, 1) = Eigen::VectorXd::Ones(k);
	A.block(k, 0, k, 1) = -X.col(2);
	A.block(k, 2, k, 1) = X.col(0);
	A.block(k, 4, k, 1) = Eigen::VectorXd::Ones(k);
	A.block(2*k, 0, k, 1) = X.col(1);
	A.block(2*k, 1, k, 1) = -X.col(0);
	A.block(2*k, 5, k, 1) = Eigen::VectorXd::Ones(k);

	// Construct XP = [X1 - P1][X2 - P2] [X3 - P3]
	Eigen::VectorXd XP, XP1, XP2, XP3;
	XP1 = X.col(0) - P.col(0);
	XP2 = X.col(1) - P.col(1);
	XP3 = X.col(2) - P.col(2);
	XP.resize(XP1.rows() + XP2.rows() + XP3.rows());
	XP << XP1, XP2, XP3;

	// Solve Wu = Z to get the optimal u
	// Z = -N*XP, W = N*A
	Eigen::VectorXd u(6);
	Eigen::MatrixXd W;
	Eigen::VectorXd Z;
	Eigen::MatrixXd NA;
	NA = diag_N * A;
	W = NA.transpose()*NA;
	Z = NA.transpose()*(diag_N*XP);
	u = W.colPivHouseholderQr().solve(-Z);

	// Compute optimal R
	Eigen::MatrixXd M = Eigen::MatrixXd::Identity(3, 3);
	M(0, 1) = -u(2);
	M(0, 2) = u(1);
	M(1, 0) = u(2);
	M(1, 2) = -u(0);
	M(2, 0) = -u(1);
	M(2, 1) = u(0);
	closest_rotation(M.transpose(), R);

	// Compute optimal t
	t(0) = u(3);
	t(1) = u(4);
	t(2) = u(5);
}
