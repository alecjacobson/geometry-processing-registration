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
	// Replace with your code
	D.resize(X.rows());
	P.resizeLike(X);
	N.resizeLike(X);

	//Get all the normals per face of the mesh
	Eigen::MatrixXd NY;
	igl::per_face_normals(VY, FY, NY);

	Eigen::RowVector3d point;
	double distance;	
	for (int i = 0; i < X.rows(); i++) {

		D(i) = (double)INFINITY;

		//Loop through all faces and find the closest point 
		for (int j = 0; j < FY.rows(); j++) {
			point_triangle_distance(X.row(i), VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), distance, point);
			
			if (distance < D(i)) {
				D(i) = distance;
				P.row(i) = point;
				N.row(i) = NY.row(j);
			}
		}
	}

	

}
