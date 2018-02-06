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
  D = Eigen::VectorXd(X.rows());
  //for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  //D = (X-P).rowwise().norm();

  double d;
  Eigen::RowVector3d p;
  

  //std::cout << "inside point mesh distance help me" << std::endl;
  for (int i = 0; i < X.rows(); i++)
  {
	  double min_dist = 1000000000000;
	  Eigen::RowVector3d closest_point(0,0,0);
	  Eigen::RowVector3d normal(0,0,0);

	  for (int j = 0; j < FY.rows(); j++)
	  {
		  int v1_index = FY.row(j)(0);
		  int v2_index = FY.row(j)(1);
		  int v3_index = FY.row(j)(2);
		  point_triangle_distance(X.row(i), VY.row(v1_index), VY.row(v2_index), VY.row(v3_index), d, p);
		  //std::cout << "min dist before: " << min_dist << std::endl;
		  //std::cout << "p before: " << closest_point << std::endl;
		  if (d < min_dist)
		  {
			  min_dist = d;
			  closest_point = p;
			  Eigen::RowVector3d edge_1 = VY.row(v2_index) - VY.row(v1_index);
			  Eigen::RowVector3d edge_2 = VY.row(v3_index) - VY.row(v2_index);
			  normal = edge_1.cross(edge_2);
			  normal.normalize();
			  //std::cout << "min dist after: " << min_dist << std::endl;
			  //std::cout << "p: " << p << std::endl;
		  }
	  }


	  D(i) = min_dist;
	  P.row(i) << closest_point;
	  N.row(i) << normal;

  }
}
