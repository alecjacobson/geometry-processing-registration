#include "point_to_plane_rigid_matching.h"
#include <igl/polar_svd.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include "closest_rotation.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // // Replace with your code
  // R = Eigen::Matrix3d::Identity();
  // t = Eigen::RowVector3d::Zero();

	// Based on taking the derivative of the equation to be minimized in the notes,
	// the only difference here should be factor of two in the numerator of the
	// right-hand side...although this seems suspicious given that the vector of
	// normals N would remain unused. But the matrix that contains those diagonal
	// matrices is a constant w.r.t. the derivative with u, so it should just go
	// outside of the derivative. And since that matrix cannot be 0, it can be
	// ignored.

	// Extract the columns of X and P
	Eigen::MatrixXd A, X1, X2, X3, P1, P2, P3, diffVec, diffVec1, diffVec2, diffVec3, Ivec;
	X1 = X.col(0);
	X2 = X.col(1);
	X3 = X.col(2);
	P1 = P.col(0);
	P2 = P.col(1);
	P3 = P.col(2);

	// Compute the vector Xi - Pi
	diffVec1 = X1 - P1;
	diffVec2 = X2 - P2;
	diffVec3 = X3 - P3;

	diffVec.resize(3*X.rows(), 1);
	diffVec.block(0, 0, X.rows(), 1) = diffVec1;
	diffVec.block(X.rows(), 0, X.rows(), 1) = diffVec2;
	diffVec.block(2*X.rows(), 0, X.rows(), 1) = diffVec3;

	// Create the big matrix A
	Ivec.setOnes(X.rows(), 1);
	A.resize(3*X.rows(), 3*X.cols()+3);
	A.setZero(3*X.rows(), 3*X.cols()+3);

	A.block(0, 1, X.rows(), 1) = X3;
	A.block(X.rows(), 0, X.rows(), 1) = -X3;
	A.block(2*X.rows(), 0, X.rows(), 1) = X2;
	A.block(0, 2, X.rows(), 1) = -X2;
	A.block(X.rows(), 2, X.rows(), 1) = X1;
	A.block(2*X.rows(), 1, X.rows(), 1) = -X1;
	A.block(0, 3, X.rows(), 1) = Ivec;
	A.block(X.rows(), 4, X.rows(), 1) = Ivec;
	A.block(2*X.rows(), 5, X.rows(), 1) = Ivec;

	// Compute A^T*A
	Eigen::MatrixXd bigA = A.transpose()*A;

	// Compute the right hand side
	Eigen::VectorXd b = -2.0*A.transpose()*diffVec;

	// Solve the system
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QRsolver(bigA);
	Eigen::VectorXd soln = QRsolver.solve(b);

	double alpha = soln(0);
	double beta = soln(1);
	double gamma = soln(2);
	t(0) = soln(3);
	t(1) = soln(4);
	t(2) = soln(5);

	Eigen::Matrix3d M;

	M(0,0) = 1.0;
	M(1,1) = 1.0;
	M(2,2) = 1.0;
	M(0,1) = -gamma;
	M(0,2) = beta;
	M(1,0) = gamma;
	M(2,0) = -beta;
	M(1,2) = -alpha;
	M(2,1) = alpha;

	closest_rotation(M, R);


	return;

}

