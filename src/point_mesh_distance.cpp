#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <limits>
#include <iostream>
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
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(), X.cols());
  Eigen::MatrixXd AllN = Eigen::MatrixXd::Zero(FY.rows(), FY.cols());
  igl::per_face_normals(VY, FY, AllN);
  //for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  //D = (X-P).rowwise().norm();
  D = Eigen::VectorXd(X.rows());

  for (int i = 0; i < X.rows(); i++) {
	  double d; Eigen::RowVector3d p;
	  D(i) = std::numeric_limits<double>::infinity();
	  for (int j = 0; j < FY.rows(); j++) {
		  point_triangle_distance(X.row(i), VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), d, p);
		  if (d < D(i)) {
			  D(i) = d;
			  P(i, 0) = p(0); P(i, 1) = p(1); P(i, 2) = p(2);
			  N.row(i) = AllN.row(j);
		  }
	  }
  }


}
