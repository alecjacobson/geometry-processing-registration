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
  
  // Project on plane spanned by triangle and see if it lies in it
  // Consulted formula at https://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle 

  // Project x on plane containing the triangle
  Eigen::RowVector3d u = b - a;
  Eigen::RowVector3d v = c - a;
  Eigen::RowVector3d tri_norm = u.cross(v);
  tri_norm = tri_norm / tri_norm.norm();
  Eigen::RowVector3d w = x - a;
  double gamma = (u.cross(w)).dot(tri_norm);
  double beta = (w.cross(v)).dot(tri_norm);
  double alpha = 1 - gamma - beta;

  // if projection lies outside triangle, project on different lines of the triangle

  if (alpha > 1 || alpha < 0 || beta > 1 || beta < 0 || gamma > 0 || gamma < 1) {
 	// consulted formula at https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line 
	double proj_ab = (a - x).dot(u) / (u.norm() * u.norm()); 
	double proj_ac = (a - x).dot(v) / (v.norm() * v.norm());
	double proj_bc = (b - x).dot(v-u) / ((v-u).norm() * (v-u).norm());
        proj_ab = std::min(std::max(proj_ab, 0.0), 1.0);
        proj_ac = std::min(std::max(proj_ac, 0.0), 1.0);
        proj_bc = std::min(std::max(proj_bc, 0.0), 1.0);
  	double dist_ab = ((a-x) - (proj_ab * u)).norm();
  	double dist_ac = ((a-x) - (proj_ac * v)).norm();
  	double dist_bc = ((b-x) - (proj_bc * (v-u))).norm();
	double min_dist = std::min(std::min(dist_ab, dist_ac), dist_bc);
	if (min_dist == dist_ac) {
		d = dist_ac;
		p = proj_ab * u + a;
	}
	if (min_dist == dist_ac) {
		d = dist_ac;
		p = proj_ac * v + a;
	}
	if (min_dist == dist_bc) {
		d = dist_bc;
		p = proj_bc * (v-u) + b;
	}
  } 
  else { 
  	p = alpha * a + beta * b + gamma * c;
  	d = (x - p).norm();
  }
}
