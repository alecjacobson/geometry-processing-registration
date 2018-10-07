#include "point_mesh_distance.h"

#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

void point_mesh_distance(const Eigen::MatrixXd & X, const Eigen::MatrixXd & VY,
		const Eigen::MatrixXi & FY, Eigen::VectorXd & D, Eigen::MatrixXd & P,
		Eigen::MatrixXd & N) {
	// Replace with your code
	P.resizeLike(X);
	N = Eigen::MatrixXd::Zero(X.rows(), X.cols());

	// normals
	Eigen::MatrixXd NY;
	igl::per_face_normals(VY, FY, NY);

	D.resize(X.rows());
	for (int i = 0; i < X.rows(); i++) {
		Eigen::RowVector3d x = X.row(i);

		double dmin = 10000;
		Eigen::RowVector3d pmin;
		Eigen::RowVector3d nmin;
		for (int j = 0; j < FY.rows(); j++) {
			double d;
			Eigen::RowVector3d p;
			Eigen::RowVector3d a = VY.row(FY(j, 0));
			Eigen::RowVector3d b = VY.row(FY(j, 1));
			Eigen::RowVector3d c = VY.row(FY(j, 2));
			point_triangle_distance(x, a, b, c, d, p);

			if (d < dmin) {
				dmin = d;
				pmin = p;
				nmin = NY.row(j);
			}

			D[i] = dmin;
			P.row(i) = pmin;
			N.row(i) = nmin;
		}
	}
}

