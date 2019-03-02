#include "point_triangle_distance.h"
#include <Eigen/Dense>

void point_triangle_distance(
	const Eigen::RowVector3d & x,
	const Eigen::RowVector3d & a,
	const Eigen::RowVector3d & b,
	const Eigen::RowVector3d & c,
	double & d,
	Eigen::RowVector3d & p)
{
	// calculate normal vector
	Eigen::RowVector3d normal = (a - b).cross(a - c);
	normal.normalize();

	// project x on to triangle plane
	Eigen::RowVector3d px = (x - a).dot(normal) * normal + x;

	/* 
		1 |6 / 3
		--A-C---
		4 |/ 5
		  B
		 /|
		/2|      */
	// if following value is positive, then px is in the triangle side of edge
	double in_ab = (a - px).cross(a - b).dot(normal);
	double in_bc = (b - px).cross(b - c).dot(normal);
	double in_ca = (c - px).cross(c - a).dot(normal);

	// find p
	if (in_ab >= 0 && in_bc >= 0 && in_ca >= 0) {
		p = px; // inside
	} else if (in_ab < 0 && !in_ca < 0) {
		p = a; // case 1
	} else if (in_ab < 0 && in_bc < 0) {
		p = b; // case 2
	} else if (in_bc < 0 && in_ca < 0) {
		p = c; // case 3
	} else if (in_ab < 0) {
		p = (a - px).dot(a - b) * (a - b) + a; // case 4
	} else if (in_bc < 0) {
		p = (b - px).dot(b - c) * (b - c) + b; // case 5
	} else if (in_ca < 0) {
		p = (c - px).dot(c - a) * (c - a) + c; // case 6
	}

	d = (x - p).norm();
}
