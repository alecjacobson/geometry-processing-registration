#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
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

  // =======================
	// RIP
	// =======================
  // My code (follow alec's instruction, but I don't know why it is not working)
  // Derivation:
  // Let N = [diagN1, diagN2, diagN3]
  // Let D = [X-P]
  // solve linear system NA u = -ND
	// int k = X.rows();
	// Eigen::MatrixXd N, A;
	// Eigen::VectorXd D, u;
	// N.resize(k ,3*k);
	// A.resize(3*k, 6);
	// D.resize(3*k);
	// u.resize(6);

	// for (int ii = 0; ii < X.rows(); ii++){
	// 	// assemble D matrix
	// 	D(3*ii  ) = X(ii,0) - P(ii,0);
	// 	D(3*ii+1) = X(ii,1) - P(ii,1);
	// 	D(3*ii+2) = X(ii,2) - P(ii,2);

	// 	// assemble A matrix
	// 	A.row(3*ii  ) <<        0,  X(ii,2),-X(ii,1), 1, 0, 0;
	// 	A.row(3*ii+1) << -X(ii,2),       0,  X(ii,0), 0, 1, 0;
	// 	A.row(3*ii+2) <<  X(ii,1), -X(ii,0),
	// 	       0, 0, 0, 1;
	// 	// assemble N matrix
	// 	N(ii,ii) = PN(ii,0);
	// 	N(ii,ii+k) = PN(ii,1);
	// 	N(ii,ii+2*k) = PN(ii,2);
	// }
	// Eigen::MatrixXd LHS = N*A;
	// Eigen::VectorXd RHS = -N*D;
	// // u = LHS.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RHS);
	// // u = LHS.colPivHouseholderQr().solve(RHS);

	// Eigen::Matrix3d M; // offdiagonal signs need to be flipped??
	// M <<     1, u(2), -u(1),
	// 		 -u(2),    1,  u(0),
	// 		  u(1),-u(0),    1;
	// closest_rotation(M, R); 
	// t(0) = u(3);
	// t(1) = u(4);
	// t(2) = u(5);
	// =======================
	// END RIP
	// =======================

	// My code (follow the original paper)
	// Reference: https://www-new.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
	int k = X.rows();
	// construct b
	Eigen::VectorXd RHS, u;
	Eigen::MatrixXd LHS;
	RHS.resize(k);
	LHS.resize(k, 6);
	u.resize(6);
	for (int ii = 0; ii < k; ii++){
		// assemble RHS
		RHS(ii) = N(ii,0)*(P(ii,0) - X(ii,0)) + 
							N(ii,1)*(P(ii,1) - X(ii,1)) + 
							N(ii,2)*(P(ii,2) - X(ii,2));
		// assemble A
		LHS.row(ii) << N(ii,2)*X(ii,1) - N(ii,1)*X(ii,2),
								 	 N(ii,0)*X(ii,2) - N(ii,2)*X(ii,0),
								 	 N(ii,1)*X(ii,0) - N(ii,0)*X(ii,1),
								 	 N(ii,0),
								 	 N(ii,1),
								 	 N(ii,2);
	}
	u = LHS.colPivHouseholderQr().solve(RHS);
	Eigen::Matrix3d M; 
	M <<     1, -u(2), u(1),
			 u(2),    1, -u(0),
			 -u(1),u(0),    1;
	closest_rotation(M, R); 
	t(0) = u(3);
	t(1) = u(4);
	t(2) = u(5);
}
