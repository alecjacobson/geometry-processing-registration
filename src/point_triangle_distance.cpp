#include "point_triangle_distance.h"
#include <math.h>
#include <Eigen/Dense>
#include <igl/doublearea.h>
#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  //Edges
  Eigen::RowVector3d edge_1 = b - a;
  Eigen::RowVector3d edge_2 = a - c;
  Eigen::RowVector3d edge_3 = c - b;

  //std::cout << "point triangle distance function" << std::endl;

  // Project x onto the plane containing the triangle (a,b,c)
  Eigen::RowVector3d plane_normal = edge_1.cross(edge_3);
  plane_normal.normalize();
  Eigen::RowVector3d projection = x - x.dot(plane_normal) * plane_normal;

  // Compute barycentric coordinates
  Eigen::MatrixXd dblA;
  Eigen::MatrixXd V(4,3);
  Eigen::MatrixXd F(4,3);
  V.row(0) << a;
  V.row(1) << b;
  V.row(2) << c;
  V.row(3) << projection;

  F.row(0) << 0,1,2;
  F.row(1) << 2,0,3;
  F.row(2) << 0,1,3;
  F.row(3) << 1,2,3;

  igl::doublearea(V, F, dblA);

  double area_t = dblA(0);
  double u = dblA(1) / area_t; 
  double v = dblA(2) / area_t; 
  double w = dblA(3) / area_t;

  //std::cout << "do i know how barycentric coordinates work?" << std::endl;

  if (u + v + w == 1)
  {
	  p = projection-x;
	  d = (projection-x).norm();
  }
  else
  {
	  Eigen::RowVector3d ap = x - a;
	  Eigen::RowVector3d bp = x - b;
	  Eigen::RowVector3d cp = x - c;
	  Eigen::RowVector3d edge_1_projection = a + (edge_1.dot(ap) / edge_1.dot(edge_1)) * edge_1;
	  Eigen::RowVector3d edge_2_projection = c + (edge_2.dot(cp) / edge_2.dot(edge_2)) * edge_2;
	  Eigen::RowVector3d edge_3_projection = b + (edge_3.dot(bp) / edge_3.dot(edge_3)) * edge_3;
	  double dist_1 = (x - edge_1_projection).norm();
	  double dist_2 = (x - edge_2_projection).norm();
	  double dist_3 = (x - edge_3_projection).norm();
	  double dist_4 = (x-a).norm();
	  double dist_5 = (x-b).norm();
	  double dist_6 = (x-c).norm();
	  double distances[] = { dist_1, dist_2, dist_3 , dist_4, dist_5, dist_6 };
	  d = *std::min_element(distances+3, distances + 6);
	  if (d == dist_1)
		  p = edge_1_projection-x;
	  else if (d == dist_2)
		  p = edge_2_projection-x;
	  else if (d == dist_3)
		  p = edge_3_projection-x;
	  else if (d == dist_4)
		  p = a;
	  else if (d == dist_5)
		  p = b;
	  else
		  p = c;
  }
}
