#include "point_triangle_distance.h"
#include <cmath>
#include <iostream>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  Eigen::Vector3d va(a(0), a(1), a(2));
  Eigen::Vector3d vb(b(0), b(1), b(2));
  Eigen::Vector3d vc(c(0), c(1), c(2));
  Eigen::Vector3d vx(x(0), x(1), x(2));
  
  Eigen::Matrix3d A;
  Eigen::Vector3d v1, v2, v3;
  v1 = vb - va; v2 = vc - va; v3 = vb - vc;
  Eigen::Vector3d n = v1.cross(v2);
  A.col(0) = v1;
  A.col(1) = v2;
  A.col(2) = n;

  // Find the coefficients of the point vx using
  // basis vectors v1, v2, n
  Eigen::BiCGSTAB<Matrix3d> solver;
  solver.compute(A);
  Eigen::Vector3d weight = solver.solve(vx - va);
  double alpha = weight(0);
  double beta = weight(1);
  double gamma = weight(2);
  Eigen::Vector3d xp = alpha * v1 + beta * v2 + va;
  // Project x to the plane of triangle abc
  xp = vx + (n.dot(va - vx) / n.norm())* n;
  Eigen::Vector3d closest = xp;
  double w1 = (v1.dot(xp - va) / v1.norm());
  double w2 = (v2.dot(xp - va) / v2.norm());
  double w3 = (v3.dot(xp - vc) / v3.norm());
  if (alpha + beta <= 1 && alpha >= 0 && beta >= 0) {
		  // Projection of x lies inside of triangle
		  //xp = alpha * v1 + beta * v2;
  }
  else {
	  // Calculate the distance from x to 3 vertices
	  // and keep the minimum
	  double da = (xp - va).norm();
	  d = da;  closest = va;
	  double db = (xp - vb).norm();
	  if (db < d) {
		  d = db;  closest = vb;
	  }
	  double dc = (xp - vc).norm();
	  if (dc < d) {
		  d = dc;  closest = vc;
	  }

	  // Calculate the distance from x to 3 edges
	  // and keep the minimum
	  if (w1 > 0 && w1 < 1) {
		  Eigen::Vector3d e1 = va + (v1.dot(xp - va) / v1.norm()) * v1;
		  double d1 = (e1 - va).norm();
		  if (d1 < d) {
			  d = d1; closest = e1;
		  }
	  }
	  if (w2 > 0 && w2 < 1) {
		  Eigen::Vector3d e2 = va + (v2.dot(xp - va) / v2.norm()) * v2;
		  double d2 = (e2 - va).norm();
		  if (d2 < d) {
			  d = d2; closest = e2;
		  }
	  }
	  if (w3 > 0 && w3 < 1) {
		  Eigen::Vector3d e3 = vc + (v3.dot(xp - vc) / v3.norm()) * v3;
		  double d3 = (e3 - va).norm();
		  if (d3 < d) {
			  d = d3; closest = e3;
		  }
	  }
	  
  }

  p = closest.transpose();
  d = (vx - closest).norm();
  
}

