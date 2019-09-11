#include "point_mesh_distance.h"
#include <igl/per_face_normals.h>
#include "point_triangle_distance.h"

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());

  // Compute normal for all input triangles
  Eigen::MatrixXd NY;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), NY);

  // Compute shortest distance for each query point
  for (int i = 0; i < X.rows(); i++) {
	  double shortest = -1;
	  double dis;
	  int triangle_idx;
	  Eigen::RowVector3d curPoint;
	  Eigen::RowVector3d shortPoint;
	  // Find the triangle with the shortest distance
	  for (int j = 0; j < FY.rows(); j++) {
		  point_triangle_distance(X.row(i), 
			  VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), dis, curPoint);
		  if (dis < shortest || shortest == -1) {
			  shortest = dis;
			  shortPoint = curPoint;
			  triangle_idx = j;
		  }
	  }
	  D(i) = shortest;
	  P.row(i) = shortPoint;
	  N.row(i) = NY.row(triangle_idx);
  }
}
