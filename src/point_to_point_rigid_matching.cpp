#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>

#include "closest_rotation.h"

void point_to_point_rigid_matching(const Eigen::MatrixXd & X,
		const Eigen::MatrixXd & P, Eigen::Matrix3d & R,
		Eigen::RowVector3d & t) {
	// Replace with your code
	// R = Eigen::Matrix3d::Identity();
	t = Eigen::RowVector3d::Zero();

	// R * PL = PR
	std::cout << X.colwise().mean() << std::endl;
	Eigen::RowVector3d PLavg = X.colwise().mean();
	Eigen::RowVector3d PRavg = P.colwise().mean();

	// subtranct mean
	Eigen::MatrixXd PL, PR;
	PL.resizeLike(X);
	PR.resizeLike(X);
	for (int i = 0; i < X.rows(); i++) {
		PL.row(i) = X.row(i) - PLavg;
		PR.row(i) = P.row(i) - PRavg;
	}

	Eigen::Matrix3d tmp = PL.transpose() * PR;
	closest_rotation(tmp, R);

	t = (PRavg.transpose() - R * PLavg.transpose()).transpose();
}

