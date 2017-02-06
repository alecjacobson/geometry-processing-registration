#include "point_triangle_distance.h"

#include <igl/barycentric_coordinates.h>

#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
	//Find closest point on plane defined by triangle
	Eigen::RowVector3d n = (b - a).cross(c - a);
	n.normalize();
	Eigen::RowVector3d point = x + n.dot(x-a)*n;

	//Form the linear system described here https://en.wikipedia.org/wiki/Barycentric_coordinate_system
	Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
	R.col(0) << a(0), a(1), a(2), -1;
	R.col(1) << b(0), b(1), b(2), -1;
	R.col(2) << c(0), c(1), c(2), -1;
	R.col(3) << 1, 1, 1, 1;
	R.row(3) << 1, 1, 1, 0;	
	
	//Directly solve the system ... should be fine for performance?
	Eigen::Vector4d point_b;
	point_b = R.inverse() * point.transpose().homogeneous();

	//Clamp and normalize the coordinates
	point_b(3) = 0;
	point_b = point_b.array().max(0);	//Clamp to min of 1
	point_b /= point_b.sum();

	//Return closest point and distance
	p = point_b(0) * a + point_b(1) * b + point_b(2) * c;	
	d = (x - p).norm();
	
	
	
}
