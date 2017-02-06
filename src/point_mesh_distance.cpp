#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
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
	D.resize(X.rows());
	P.resizeLike(X);
	N.resizeLike(X);

	MatrixXd facenormals;
	igl::per_face_normals(VY, FY, facenormals);

	for (int j = 0; j < X.rows(); ++j)
	{
		auto x = X.row(j);
		double mindistance = DBL_MAX;
		int closestFace = -1;

		//Iterate over all faces to find the closest face
		for (int i = 0; i < FY.rows(); ++i)
		{
			auto faces = FY.row(i);
			auto p1 = VY.row(faces(0)), p2 = VY.row(faces(1)), p3 = VY.row(faces(2));
			double distance;
			RowVector3d closestPoint;
			point_triangle_distance(x, p1, p2, p3, distance, closestPoint);
			if (distance < mindistance)
			{
				mindistance = distance;
				closestFace = i;
			}
		}

		assert(closestFace >= 0);

		//Give the index of the closest face compute normal
		auto faces = FY.row(closestFace);
		Vector3d p1 = VY.row(faces(0)), p2 = VY.row(faces(1)), p3 = VY.row(faces(2));
		double distance;
		RowVector3d closestPoint;
		point_triangle_distance(x, p1, p2, p3, distance, closestPoint);
		Vector3d normal = (p2 - p1).cross(p3 - p1);
		normal.normalize();
	
		//Populate return values
		D(j) = distance;
		P.row(j) = closestPoint;
		N.row(j) = facenormals.row(closestFace);
	}
}
