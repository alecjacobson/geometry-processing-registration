#include "closest_rotation.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(const Eigen::Matrix3d & M, Eigen::Matrix3d & R) {
	// Replace with your code
	R = Eigen::Matrix3d::Identity();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M,
			Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
	// Matrix3d S = U.inverse() * M * V.transpose().inverse(); // S = U^-1 * A * VT * -1

	R(2, 2) = (V * U.transpose()).determinant();
	R = V * R * U.transpose();
}

