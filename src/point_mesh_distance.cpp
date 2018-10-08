#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

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
  Eigen::MatrixXd NY = Eigen::MatrixXd::Zero(FY.rows(), 3);
  igl::per_face_normals(VY, FY, Eigen::Vector3d::Zero(), NY);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  Eigen::RowVectorXd distances(FY.rows());
  Eigen::MatrixXd points(FY.rows(), 3);

  for (int i = 0; i < X.rows(); i++) {
	Eigen::RowVector3d x = X.row(i);
        double d = 0;
        Eigen::RowVector3d p;

        // Find closest point of x to each face 
        for (int j = 0; j < FY.rows(); j++) {
		Eigen::RowVector3d a = VY.row(FY(j,0));
		Eigen::RowVector3d b = VY.row(FY(j,1));
		Eigen::RowVector3d c = VY.row(FY(j,2));
	 	point_triangle_distance(x, a, b, c, d, p);
		distances(j) = d;
                points.row(j) = p;
        }
        
	// Get distance, normal, and point information from that search
        D(i) = distances.minCoeff();
        for (int j = 0; j < FY.rows(); j++) {
		if (distances(j) == D(i)) {
			P.row(i) = points.row(j);
			N.row(i) = NY.row(j); 
		}
        }
  }
}
