#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <Eigen/LU>
#include "igl/per_face_normals.h"

using namespace Eigen;

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
	int pointCount = X.rows();
	int vertexCount = VY.rows();
	int triangleCount = FY.rows();

	P.resizeLike(X);

	Eigen::MatrixXd FN(triangleCount, 3);
	igl::per_face_normals(VY, FY, FN);

	D.resize(X.rows());
	N = Eigen::MatrixXd::Zero(X.rows(), X.cols());
	for (int i = 0; i < pointCount; i++) {

		Vector3d x = X.row(i);

		double closestD = 9999999999999.0;
		RowVector3d closestP;
		RowVector3d closestN;
		for (int j = 0; j < triangleCount; j++) {

			int aIndex = FY(j, 0);
			int bIndex = FY(j, 1);
			int cIndex = FY(j, 2);
			Vector3d a = VY.row(aIndex);
			Vector3d b = VY.row(bIndex);
			Vector3d c = VY.row(cIndex);

			RowVector3d p;
			double d;
			point_triangle_distance(x, a, b, c, d, p);

			if (d < closestD) {
				closestP = p;
				closestD = d;
				closestN = FN.row(j);
			}
		}

		D(i) = closestD;
		P.row(i) = closestP;
		N.row(i) = closestN;
	}
}
