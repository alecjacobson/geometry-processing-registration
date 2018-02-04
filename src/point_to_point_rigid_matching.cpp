#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <iostream>
using namespace std;

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();

	// =======================
	// RIP
	// =======================
  // My code (approximate solution)
 //  Eigen::MatrixXd A;
 //  Eigen::MatrixXd PX;
 //  A.resize(3*X.rows(), 6);
 //  PX.resize(3*X.rows(), 1);

 //  Eigen::RowVector3d Xpt, Ppt;
	// for (int ii = 0; ii < X.rows(); ii++){
	// 	Xpt = X.row(ii);
	// 	Ppt = P.row(ii);

	// 	// assemble PX
	// 	PX(3*ii  ) = Xpt(0) - Ppt(0);
	// 	PX(3*ii+1) = Xpt(1) - Ppt(1);
	// 	PX(3*ii+2) = Xpt(2) - Ppt(2);

	// 	// assemble A
	// 	A.row(3*ii  ) << 0, Xpt(2), -Xpt(1), 1, 0, 0;
	// 	A.row(3*ii+1) << -Xpt(2), 0, Xpt(0), 0, 1, 0;
	// 	A.row(3*ii+2) << Xpt(1), -Xpt(0), 0, 0, 0, 1;
	// }  
	
	// // compute u
	// Eigen::VectorXd u = (A.transpose()*A).inverse() * (-A.transpose()*PX);
	// // compute R
	// Eigen::Matrix3d M;
	// M <<     1,-u(2), u(1),
	// 		  u(2),    1,-u(0),
	// 		 -u(1), u(0),    1;
	// closest_rotation(M, R);
	// // compute t
	// t(0) = u(3);
	// t(1) = u(4);
	// t(2) = u(5);
	// =======================
	// END RIP
	// =======================

	// My code (closed form)
	Eigen::MatrixXd Xbar, Pbar;
	Eigen::Matrix3d M;
	Eigen::RowVector3d Xmean, Pmean;

	Xmean = X.colwise().sum() / X.rows();
	Pmean = P.colwise().sum() / P.rows();

	Xbar = X.rowwise() - Xmean;
	Pbar = P.rowwise() - Pmean;

	M = (Xbar.transpose() * Pbar).transpose(); // in the README, this is a typo
	closest_rotation(M, R);
	t = Pmean - (R*Xmean.transpose()).transpose();
}

