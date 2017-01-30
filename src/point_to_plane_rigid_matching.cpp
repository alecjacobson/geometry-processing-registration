#include "point_to_plane_rigid_matching.h"
#include <Eigen/QR>
void point_to_plane_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	const Eigen::MatrixXd & N,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	//The derivation is a mess, but problem reduces to solving a 6D linear system. 
	//Nice and easy. 
	int k = X.rows();
	Eigen::MatrixXd A(3 * k, 6);
	Eigen::VectorXd J(3 * k); //This is the collection of diagonalized normals
	Eigen::MatrixXd K(k, 3 * k); //This is the X-P collection.
	
	//TODO: Create all these matrices.
	A.setZero();
	A.block(k, 0, k, 1) = -X.col(2);
	A.block(2 * k, 0, k, 1) = X.col(1);
	A.block(0, 1, k, 1) = X.col(2);
	A.block(2 * k, 1, k, 1) = -X.col(0);
	A.block(0, 2, k, 1) = -X.col(1);
	A.block(k, 2, k, 1) = X.col(0);
	A.block(0, 3, k, 1) = Eigen::VectorXd::Ones(k);
	A.block(k, 4, k, 1) = Eigen::VectorXd::Ones(k);
	A.block(2 * k, 5, k, 1) = Eigen::VectorXd::Ones(k);

	J.segment(0, k) = X.col(0) - P.col(0);
	J.segment(k, k) = X.col(1) - P.col(1);
	J.segment(k * 2, k) = X.col(2) - P.col(2);

	K.setZero();
	K.block(0, 0, k, k) = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(N.col(0));
	K.block(0, k, k, k) = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(N.col(1));
	K.block(0, 2*k, k, k) = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(N.col(2));

	auto M = A.transpose()*K.transpose()*K*A;
	Eigen::VectorXd u = M.colPivHouseholderQr().solve(A.transpose()*K.transpose()*K*J);

	R = Eigen::Matrix3d::Zero();
	R(1, 0) = -u(2); R(0, 1) = u(2);
	R(2, 0) = u(1); R(0, 2) = -u(1);
	R(1, 2) = -u(0); R(2, 1) = u(0);
	t = u.segment<3>(3);
}
