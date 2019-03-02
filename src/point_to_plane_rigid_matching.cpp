#include "point_to_plane_rigid_matching.h"
#include <iostream>
#include <Eigen/Dense>
#include <closest_rotation.h>

using namespace std;

void point_to_plane_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	const Eigen::MatrixXd & N,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	// Replace with your code
	R = Eigen::Matrix3d::Identity();
	t = Eigen::RowVector3d::Zero();
	int k = X.rows();
	Eigen::MatrixXd A(3 * k, 6);
	A.setZero();
	Eigen::VectorXd kones(k);
	kones.setOnes();
	A.block(k,   0, k, 1) = -X.col(2);
	A.block(2*k, 0, k, 1) =  X.col(1);
	A.block(0,   1, k, 1) =  X.col(2);
	A.block(2*k, 1, k, 1) = -X.col(0);
	A.block(0,   2, k, 1) = -X.col(1);
	A.block(k,   2, k, 1) =  X.col(0);

	A.block(0,   3, k, 1) = kones;
	A.block(k,   4, k, 1) = kones;
	A.block(2*k, 5, k, 1) = kones;

	Eigen::VectorXd xp(3 * k);
	xp.segment(0,   k) = X.col(0) - P.col(0);
	xp.segment(k,   k) = X.col(1) - P.col(1);
	xp.segment(2*k, k) = X.col(2) - P.col(2);

	Eigen::MatrixXd DN1 = N.col(0).asDiagonal();
	Eigen::MatrixXd DN2 = N.col(1).asDiagonal();
	Eigen::MatrixXd DN3 = N.col(2).asDiagonal();

	Eigen::MatrixXd DN(k, 3 * k);
	DN.block(0,   0, k, k) = DN1;
	DN.block(0,   k, k, k) = DN2;
	DN.block(0, 2*k, k, k) = DN3;

	Eigen::MatrixXd DNA = DN * A;
	// Least square fit
	Eigen::VectorXd u = (DNA.transpose() * DNA).ldlt().solve(DNA.transpose() *DN* (-xp));
	t = u.segment(3, 3).transpose();
	double alpha = u(0);
	double beta  = u(1);
	double gamma = u(2);
	Eigen::MatrixXd M = Eigen::Matrix3d::Identity();
	M(0, 1) =  gamma;
	M(1, 0) = -gamma;
	M(0, 2) = -beta;
	M(2, 0) =  beta;
	M(1, 2) =  alpha;
	M(2, 1) = -alpha;

	closest_rotation(M, R);


}
