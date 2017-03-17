#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{

	//Make use of Voronoi regions
	
	// Check if P is in vertex region of A
	Eigen::RowVector3d ab = b - a;
	Eigen::RowVector3d ac = c - a;
	Eigen::RowVector3d ax = x - a;
	float d1 = ab.dot(ax);
	float d2 = ac.dot(ax);
	if (d1 <= 0.0f && d2 <= 0.0f) {
		p = a;
		d = (p - x).norm();
		return;
	}

	// Check if P is in vertex region of B
	Eigen::RowVector3d bx = x - b;
	float d3 = ab.dot(bx);
	float d4 = ac.dot(bx);
	if (d3 >= 0.0f && d4 <= d3) {
		p = b;
		d = (p - x).norm();
		return;
	}

	// Check if P is in edge region of AB. If so, project onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		float w = d1 / (d1-d3);
		p = a + w*ab;
		d = (p - x).norm();
		return;
	}

	// Check if P is in vertex region of C
	Eigen::RowVector3d cx = x - c;
	float d5 = ab.dot(cx);
	float d6 = ac.dot(cx);
	if (d6 >= 0.0f && d5 <= d6) {
		p = c;
		d = (p - x).norm();
		return;
	}

	// Check if P is in edge region of AC. If so, project onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		float w = d2 / (d2 - d6);
		p = a + w*ac;
		d = (p - x).norm();
		return;
	}

	// Check if P is in edge region of BC. If so, project onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4-d3) >= 0.0f && (d5-d6) >= 0.0f) {
		float w = (d4-d3) / ((d4-d3) + (d5-d6));
		p = b + w*(c-b);
		d = (p - x).norm();
		return;
	}

	// P inside face region
	float denom = 1.0f / (va + vb + vc);
	float v = vb*denom;
	float w = vc*denom;
	p = a + v*ab + w*ac;
	d = (p - x).norm();
	return;
}
