#include "point_triangle_distance.h"
#include <cmath>
#include <math.h> 
#include <iostream>
#include <igl/project_to_line_segment.h>
using namespace std;
// =======================
// RIP
// =======================
// void solveBarycentric(
// 	const Eigen::RowVector3d p, 
// 	const Eigen::RowVector3d a, 
// 	const Eigen::RowVector3d b, 
// 	const Eigen::RowVector3d c, 
// 	double &u, 
// 	double &v, 
// 	double &w)
// {
//     Eigen::RowVector3d v0 = b - a;
//     Eigen::RowVector3d v1 = c - a;
//     Eigen::RowVector3d v2 = p - a;
//     double d00 = v0.dot(v0);
//     double d01 = v0.dot(v1);
//     double d11 = v1.dot(v1);
//     double d20 = v2.dot(v0);
//     double d21 = v2.dot(v1);
//     double denom = d00 * d11 - d01 * d01;
//     v = (d11 * d20 - d01 * d21) / denom;
//     w = (d00 * d21 - d01 * d20) / denom;
//     u = 1.0 - v - w;
// }

// void point2LineIntersect(
// 	const Eigen::RowVector3d p, 
// 	const Eigen::RowVector3d a, 
// 	const Eigen::RowVector3d b, 
// 	Eigen::RowVector3d p_proj,
// 	double d)
// {
// 	Eigen::RowVector3d ap = p - a;
// 	Eigen::RowVector3d ab = b - a;
// 	double projLength = ap.dot(ab);
// 	double theta = acos(projLength / (ap.norm() * ab.norm()));
// 	p_proj = ap.norm()*cos(theta) * (ab / ab.norm()) + a;
// 	d = ap.norm() * sin(theta);
// }
// =======================
// END RIP
// =======================

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & aa,
  const Eigen::RowVector3d & bb,
  const Eigen::RowVector3d & cc,
  double & dd,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  // d = 0;
  // p = a;

	// =======================
	// RIP
	// =======================
  // My code (old implementation)
	// // find face normal
	// Eigen::RowVector3d ab = b - a;
	// Eigen::RowVector3d ac = c - a;
	// Eigen::RowVector3d N = ab.cross(ac);
	// N = N / N.norm(); // normalize N

	// // find point-plane intersection
	// // assume intersection p0 = x + t.dot(N)
	// double t = N.dot(a) - N.dot(x);
	// Eigen::RowVector3d p0 = x + t * N;

	// // solve barycentric coordinate (u,v,w)
	// double u,v,w;
	// solveBarycentric(p0, a, b, c, u, v, w);
	// // cout << "bary:" << u << v << w << endl;

	// // case: p0 is inside thr triangle
	// if (0<=u && u<=1 && 0<=v && v<=1 && 0<=w && w<=1){
	// 	p = p0; 
	// 	d = (x - p0).norm();
	// }
	// // case: only 1 is positive (closest pt is vertex)
	// else if (v<=0 && w<=0){
	// 	p = a; 
	// 	d = (x - a).norm();
	// }
	// else if (u<=0 && w<=0){
	// 	p = b; 
	// 	d = (x - b).norm();
	// }
	// else if (u<=0 && v<=0){
	// 	p = c; 
	// 	d = (x - c).norm();
	// }
	// // case: only 1 is negative (closest pt is edge)
	// else if (w<=0 && u>=0 && v>= 0){
	// 	point2LineIntersect(x, a, b, p, d);
	// }
	// else if (v<=0 && u>=0 && w>= 0){
	// 	point2LineIntersect(x, c, a, p, d);
	// }
	// else if (u<=0 && w>=0 && v>= 0){
	// 	point2LineIntersect(x, b, c, p, d);
	// }
	// =======================
	// END RIP
	// =======================

	// My code (better implementation)
	// Reference: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
	Eigen::RowVector3d B = aa;
	Eigen::RowVector3d E0 = bb - aa;
	Eigen::RowVector3d E1 = cc - aa;
	Eigen::RowVector3d P = x;

	double a = E0.dot(E0);
	double b = E0.dot(E1);
	double c = E1.dot(E1);
	double d = E0.dot(B-P);
	double e = E1.dot(B-P);
	double f = (B-P).dot(B-P);

	double s = b*e - c*d;
	double t = b*d - a*e;
	double det = a*c - b*b;

	if (s+t <= det){
		if (s < 0){
			if(t < 0){
				// ===============
				// region 4
				if (d < 0){
					t = 0;
					if (-d >= a)
						s = 1;
					else 
						s = -d / a;
				}
				else{
					s = 0;
					if (e >= 0)
						t = 0;
					else if (-e >= c)
						t = 1;
					else 
						t = -e /c;
				}
				// ===============
			}
			else {
				// ===============
				// region 3
				s = 0;
				if (e >= 0)
					t = 0;
				else if (-e >= c)
					t = 1;
				else
					t = -e / c;
				// ===============
			}
		}
		else if (t < 0){
			// ===============
			// region 5
			t = 0;
			if (d >= 0)
				s = 0;
			else if (-d >= a)
				s = 1;
			else 
				s = -d / a;
			// ===============
		}
		else {
			// ===============
			// region 0
			s /= det;
			t /= det;
			// ===============
		}
	}
	else {
		if (s < 0){
			// ===============
			// region 2
			double temp0 = b + d;
			double temp1 = c + e;
			if (temp1 > temp0){
				double num = temp1 - temp0;
				double denom = a - 2*b + c;
				if (num >= denom)
					s = 1;
				else 
					s = num / denom;
				t = 1-s;
			}
			else {
				s = 0;
				if (temp1 <= 0)
					t = 1;
				else if (e >= 0)
					t = 0;
				else
					t = -e / c;
			}
			// ===============
		}
		else if (t < 0){
			// ===============
			// region 6
			double temp0 = b + e;
			double temp1 = a + d;
			if (temp1 > temp0){
				double num = temp1 - temp0;
				double denom = a - 2*b + c;
				if (num >= denom)
					t = 1;
				else 
					t = num / denom;
				s = 1-t;
			}
			else{
				t = 0;
				if (temp1 <= 0)
					s = 1;
				else if (d >= 0)
					s = 0;
				else
					s = -d / a;
			}
			// ===============
		}
		else {
			// ===============
			// region 1
			double num = (c+e) - (b+d);
			if (num <= 0)
				s = 0;
			else{
				double denom = a - 2*b + c;
				if (num >= denom)
					s = 1;
				else
					s = num / denom;
			}
			t = 1-s;
			// ===============
		}
	}
	p = B + s*E0 + t*E1;
	dd = (p-x).norm();
}
