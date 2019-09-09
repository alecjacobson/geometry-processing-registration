#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/LU>

using namespace Eigen;

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
	int k = X.rows();
	MatrixXd A(3 * k, 6);
	A.setZero(3 * k, 6);
	VectorXd b(3 * k);

	// Set up the matrix A.
	for (int i = 0; i < k; i++) {
		A(i * 3, 1) = X(i, 2);
		A(i * 3, 2) = -X(i, 1);
		A(i * 3, 3) = 1;

		A(i * 3 + 1, 0) = -X(i, 2);
		A(i * 3 + 1, 2) = X(i, 0);
		A(i * 3 + 1, 4) = 1;

		A(i * 3 + 2, 0) = X(i, 1);
		A(i * 3 + 2, 1) = -X(i, 0);
		A(i * 3 + 2, 5) = 1;

		b(i * 3) = X(i, 0) - P(i, 0);
		b(i * 3 + 1) = X(i, 1) - P(i, 1);
		b(i * 3 + 2) = X(i, 2) - P(i, 2);
	}

	MatrixXd diagN1 = N.col(0).asDiagonal();
	MatrixXd diagN2 = N.col(1).asDiagonal();
	MatrixXd diagN3 = N.col(2).asDiagonal();
	MatrixXd diagonals(diagN1.rows(), diagN1.cols() * 3);
	diagonals << diagN1, diagN2, diagN3;

	// The algorithm is like point -> point, but with the diagonals matrix, 
	// so I just factor it in, in advance.
	A = diagonals * A;
	b = diagonals * b;

	VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * b);
	double alpha = u(0);
	double beta = u(1);
	double gamma = u(2);
	t = Eigen::RowVector3d(u(3), u(4), u(5));

	Matrix3d M = Eigen::Matrix3d::Identity();
	M(0, 1) = -gamma;
	M(0, 2) = beta;
	M(1, 0) = gamma;
	M(1, 2) = -alpha;
	M(2, 0) = -beta;
	M(2, 1) = alpha;

	M.transposeInPlace();

	closest_rotation(M, R);
}

