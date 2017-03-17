#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
	int k = X.rows();

	Eigen::MatrixXd NCat(k,3*k);
	Eigen::MatrixXd N0 = N.col(0).asDiagonal();
	Eigen::MatrixXd N1 = N.col(1).asDiagonal();
	Eigen::MatrixXd N2 = N.col(2).asDiagonal();

	//std::cout << N.rows() << std::endl;

	NCat << N0, N1, N2;

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*k,6);
	
	A.block(k, 0, k, 1)     = X.col(2);
	A.block(2 * k, 0, k, 1) = -X.col(1);

	A.block(0, 1, k, 1)     = -X.col(2);
	A.block(2 * k, 1, k, 1) = X.col(0);

	A.block(0, 2, k, 1)     = X.col(1);
	A.block(k, 2, k, 1)     = -X.col(0);
	
	A.block(0, 3, k, 1)     = Eigen::MatrixXd::Ones(k, 1);
	A.block(k, 4, k, 1)     = Eigen::MatrixXd::Ones(k, 1);
	A.block(2 * k, 5, k, 1) = Eigen::MatrixXd::Ones(k, 1);

	Eigen::MatrixXd XP(3*k,1);
	XP.block(0, 0, k, 1)     = X.col(0) - P.col(0);
	XP.block(k, 0, k, 1)     = X.col(1) - P.col(1);
	XP.block(2 * k, 0, k, 1) = X.col(2) - P.col(2);

	Eigen::MatrixXd AT = A.transpose();
	Eigen::MatrixXd Inv = (AT*A).inverse();

	Eigen::VectorXd u = -Inv*AT*XP;
	

	Eigen::Vector3d RParams = u.topRows(3);
	t = u.bottomRows(3);

	//std::cout << t << std::endl << std::endl;

	Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
	M(1, 0) = RParams(2);
	M(0, 2) = RParams(1);
	M(2, 1) = RParams(0);
	M(0, 1) = -M(1, 0);
	M(2, 0) = -M(0, 2);
	M(1, 2) = -M(2, 1);
	//M = M.inverse();

	//std::cout << M << std::endl << std::endl;
	closest_rotation(M, R);
	//R = R.inverse(); // I DON'T KNOW WHY THIS IS NEEDED. IT WAS A COMPLETE GUESS AND IT WORKS?!?!
}
