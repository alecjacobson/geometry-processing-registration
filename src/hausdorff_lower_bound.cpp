#include "hausdorff_lower_bound.h"

#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

double hausdorff_lower_bound(const Eigen::MatrixXd & VX,
		const Eigen::MatrixXi & FX, const Eigen::MatrixXd & VY,
		const Eigen::MatrixXi & FY, const int n) {
	// Replace with your code
	Eigen::MatrixXd Xsam;
	random_points_on_mesh(n, VX, FX, Xsam);

	Eigen::VectorXd D;
	Eigen::MatrixXd P, N;
	point_mesh_distance(Xsam, VY, FY, D, P, N);

	double nre = 0;
	for (int i = 0; i < n; i++) {
		if (nre < D[i])
			nre = D[i];
	}

	return nre;
}
