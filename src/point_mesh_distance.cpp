#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <Eigen/Dense>
#include <iostream>

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
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  //for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  

  for (int i = 0; i < X.rows(); ++i) { //point loop
	  Eigen::RowVector3d x = X.row(i);
	  double dMin = INT_MAX;
	  Eigen::RowVector3d pMin;
	  int jMin = 0;
	  for (int j = 0; j < FY.rows(); ++j) { // triangle loop
		  double d = 0.0f;
		  Eigen::RowVector3d p;

		  Eigen::VectorXi vIndices = FY.row(j);
		  Eigen::RowVector3d a = VY.row(vIndices(0));
		  Eigen::RowVector3d b = VY.row(vIndices(1));
		  Eigen::RowVector3d c = VY.row(vIndices(2));
		  
		  point_triangle_distance(x, a, b, c, d, p);
		  if (d < dMin) {
			  dMin = d;
			  pMin = p;
			  jMin = j;
		  }
		  if (dMin == 0) {
			  break; // break out of inner loop if x is already on p
		  }
	  }
	  P.row(i) = pMin;

	  Eigen::VectorXi vIndices = FY.row(jMin);
	  Eigen::RowVector3d a = VY.row(vIndices(0));
	  Eigen::RowVector3d b = VY.row(vIndices(1));
	  Eigen::RowVector3d c = VY.row(vIndices(2));
	  
	  Eigen::RowVector3d ba = b - a;
	  Eigen::RowVector3d ca = c- a;
	  Eigen::RowVector3d n = ba.cross(ca); // avoid computing cross product until outside the loop to save CPU
	  n.normalize();
	  //std::cout << n << std::endl << std::endl;
	  N.row(i) = n;
  }

  D = (X - P).rowwise().norm();
}
