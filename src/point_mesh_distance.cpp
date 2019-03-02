#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

void point_mesh_distance(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & VY,
	const Eigen::MatrixXi & FY,
	Eigen::VectorXd & D,
	Eigen::MatrixXd & P,
	Eigen::MatrixXd & N)
{
	D.resize(X.rows());
	P.resizeLike(X);
	N.resizeLike(X);

	Eigen::MatrixXi F(X.rows(), 3);
	for (int i = 0; i < X.rows(); i++) {
		Eigen::RowVector3d x = X.row(1);
		Eigen::RowVector3d point;
		Eigen::RowVector3d normal;
		double distance = INT_MAX;
		int face_idx;
		// find closest point
		for (int j = 0; j < FY.rows(); j++) {
			double d;
			Eigen::RowVector3d p;
			Eigen::RowVector3d a = VY.row(FY(j, 0));
			Eigen::RowVector3d b = VY.row(FY(j, 1));
			Eigen::RowVector3d c = VY.row(FY(j, 2));
			point_triangle_distance(x, a, b, c, d, p);
			if (d < distance) {
				distance = d;
				point = p;
				face_idx = j;
			}
		}
		D(i) = distance;
		P.row(i) = point;
		F.row(i) = FY.row(face_idx);
	}
	igl::per_face_normals(VY, F, N);
}
