#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

using namespace Eigen;

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
	RowVector3d n = (a - b).cross(a - c);
	n.normalize();

	//closest point on the plane of the triangle
	RowVector3d x_0 = x + n.dot(a - x) * n;

	Matrix3d v;
	v.row(0) = a;
	v.row(1) = b;
	v.row(2) = c;

	// compute barycentric coordinates
	RowVector3d bary = x * v.inverse();

	if (bary(0) < 0)
		bary(0) = 0;
	if (bary(1) < 0)
		bary(1) = 0;
	if (bary(2) < 0)
		bary(2) = 0;

	bary.normalize();

	p = bary * v;
	d = (x - p).norm();
}
