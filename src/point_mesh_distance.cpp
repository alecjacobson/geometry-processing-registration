#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

using namespace Eigen;

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
	int numF = FY.rows(), numPts = X.rows();
	D.resize(numPts);
	P.resizeLike(X);
	N.resizeLike(X);

	MatrixXd NY;
	igl::per_face_normals(VY, FY, NY);

	RowVector3d p, p_min, x;
	double d, d_min;
	int idx_min;

	// Big-O(km) search
	for (int i = 0; i < numPts; ++i)
	{
		d_min = HUGE_VAL;
		x = X.row(i).transpose();
		idx_min = -1;

		//search over all triangles
		for (int j = 0; j < numF; ++j)
		{
			// use space partitioning here if enough time
			point_triangle_distance(x, VY.row(FY(j, 0)).transpose(), VY.row(FY(j, 1)).transpose(), VY.row(FY(j, 2)).transpose(), d, p);

			if (d < d_min)
			{
				d_min = d;
				idx_min = j;
				p_min = p;
			}
		}

		assert(idx_min >= 0 && idx_min < numF);

		D(i) = d_min;
		P.row(i) = p_min.transpose();
		N.row(i) = NY.row(idx_min);
	}
}
