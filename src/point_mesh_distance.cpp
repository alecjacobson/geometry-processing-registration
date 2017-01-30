#include "point_mesh_distance.h"
#include <Eigen/Dense>
#include <limits>
#include <igl/per_face_normals.h>

void point_mesh_distance(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & VY,
	const Eigen::MatrixXi & FY,
	Eigen::VectorXd & D,
	Eigen::MatrixXd & P,
	Eigen::MatrixXd & N)
{
	//For now, do the n^2 approach. Later on, do the kd-tree approach.
	//For now, loop through each triangle, and compute the distance. 
	//We don't use the point-mesh-distance function because we can cache some matrix operations to improve perf.
	//Later on, can use a kd-tree, which can store these 4x4 matrices with the triangles.

	P.resizeLike(X);
	D.resize(X.rows());
	D.setConstant(std::numeric_limits<double>::max());
	N.resizeLike(P);

	Eigen::MatrixXd normals;
	igl::per_face_normals(VY, FY, normals);

	for (int i = 0; i < FY.rows(); i++) {
		auto a = VY.row(FY(i, 0));
		auto b = VY.row(FY(i, 1));
		auto c = VY.row(FY(i, 2));
		Eigen::Matrix4d A;
		A.row(0) = Eigen::Vector4d(a.dot(a), a.dot(b), a.dot(c), -1);
		A.row(1) = Eigen::Vector4d(b.dot(a), b.dot(b), b.dot(c), -1);
		A.row(2) = Eigen::Vector4d(c.dot(a), c.dot(b), c.dot(c), -1);
		A.row(3) = Eigen::Vector4d(1, 1, 1, 0);
		auto Ai = A.inverse();

		for (int j = 0; j < X.rows(); j++) {
			Eigen::Vector3d x = X.row(j);
			Eigen::Vector4d B = Eigen::Vector4d(a.dot(x), b.dot(x), c.dot(x), 1);
			Eigen::Vector4d bary = Ai*B;//I think at 4x4, it's faster to directly compute the inverse than it is to solve.

			bary.w() = 0;
			bary.x() = std::max(bary.x(), 0.0);
			bary.y() = std::max(bary.y(), 0.0);
			bary.z() = std::max(bary.z(), 0.0);
			bary /= bary.sum();

			Eigen::Vector3d p = bary(0)*a + bary(1)*b + bary(2)*c;
			auto dist = (x - p).norm();
			if (dist < D(j)) {
				D(j) = dist;
				P.row(j) = p;
				N.row(j) = normals.row(i);
			}
		}
	}
}
