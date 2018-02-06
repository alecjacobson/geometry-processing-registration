#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
	
	Eigen::RowVector3d v0 = x - a;
	Eigen::RowVector3d v1 = b - a;
	Eigen::RowVector3d v2 = c - a;
	Eigen::RowVector3d n = v1.cross(v2);
	n.normalize();
	double dis = abs(v0.dot(n));
	Eigen::RowVector3d x0 = x - dis * n; // Point projected to plane of the triangle
	Eigen::RowVector3d v3 = x0 - a;
	double v11 = v1.dot(v1);
	double v12 = v1.dot(v2);
	double v22 = v2.dot(v2);
	double v31 = v3.dot(v1);
	double v32 = v3.dot(v2);
	double invden = 1.0/(v22*v11 - v12*v12);
	double alpha = (v22*v31 - v12 * v32)*invden;
	double beta = (v11*v32 - v12 * v31)*invden;
	if (alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
		// Inside target triangle
		p = a + alpha*v1 + beta*v2;
		d = dis;
	}
	else if (alpha >= 0 && beta >= 0 && alpha + beta > 1) {
		// Porject x0 to line bc
		Eigen::RowVector3d v_x0c = c - x0;
		Eigen::RowVector3d v_x0b = b - x0;
		Eigen::RowVector3d v_bc = c - b;
		Eigen::RowVector3d dir = (v_x0c.cross(v_x0b)).cross(v_bc);
		dir.normalize();
		double dis_x0 = abs(v_x0b.dot(dir));
		Eigen::RowVector3d x0_p = x0 + dis_x0 * dir;
		double t = (x0_p - b).dot(v_bc) / v_bc.dot(v_bc);
		if (t > 1)
			p = c;
		else if (t < 0)
			p = b;
		else // 0 <= t <= 1
			p = x0_p;
		d = (x - p).norm();
	}
	else if (alpha >= 0 && beta < 0) {
		// Project x0 to line ab
		Eigen::RowVector3d v_x0b = b - x0;
		Eigen::RowVector3d v_x0a = a - x0;
		Eigen::RowVector3d v_ab = b - a;
		Eigen::RowVector3d dir = (v_x0b.cross(v_x0a)).cross(v_ab);
		dir.normalize();
		double dis_x0 = abs(v_x0a.dot(dir));
		Eigen::RowVector3d x0_p = x0 + dis_x0 * dir;
		double t = (x0_p - a).dot(v_ab) / v_ab.dot(v_ab);
		if (t > 1)
			p = b;
		else if (t < 0)
			p = a;
		else // 0 <= t <= 1
			p = x0_p;
		d = (x - p).norm();
	}
	else if (alpha < 0 && beta >= 0) {
		// Project x0 to line ac
		Eigen::RowVector3d v_x0a = a - x0;
		Eigen::RowVector3d v_x0c = c - x0;
		Eigen::RowVector3d v_ca = a - c;
		Eigen::RowVector3d dir = (v_x0a.cross(v_x0c)).cross(v_ca);
		dir.normalize();
		double dis_x0 = abs(v_x0c.dot(dir));
		Eigen::RowVector3d x0_p = x0 + dis_x0 * dir;
		double t = (x0_p - c).dot(v_ca) / v_ca.dot(v_ca);
		if (t > 1)
			p = a;
		else if (t < 0)
			p = c;
		else // 0 <= t <= 1
			p = x0_p;
		d = (x - p).norm();
	}
	else {// (alpha < 0 && beta < 0)
		// Closest point is a
		p = a;
		d = (x - p).norm();
	}
}
