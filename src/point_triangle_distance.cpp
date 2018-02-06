#include "point_triangle_distance.h"
#include <limits>
#include <iostream>
#include <stdlib.h>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // following algorithm from below link
  // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.104.4264&rep=rep1&type=pdf
  Eigen::RowVector3d ab = a-b;
  Eigen::RowVector3d ba = b-a;
  Eigen::RowVector3d ac = a-c;
  Eigen::RowVector3d ca = c-a;
  Eigen::RowVector3d bc = b-c;
  Eigen::RowVector3d cb = c-b;
  Eigen::RowVector3d ax = a-x;
  Eigen::RowVector3d n = ba.cross(cb);
  double area = n.norm();
  double angle = ax.dot(n) / n.norm();
  Eigen::RowVector3d projectx = x - angle * n / n.norm();
  
  Eigen::RowVector3d pb = projectx - b;
  Eigen::RowVector3d pc = projectx - c;
  Eigen::RowVector3d pa = projectx - a;

  double alpha = (pb.cross(pc)).norm() / area; 
  double beta = (pc.cross(pa)).norm() / area;
  double gamma = 1 - alpha - beta;

  if(0 <= alpha && alpha <= 1 &&
  	0 <= beta && beta <= 1 &&
  	0 <= gamma && gamma <= 1) {
  	p = projectx;
  	d = (x - p).norm();
  	return ;
  }
  d = std::numeric_limits<double>::max();
  dist_helper(x, projectx, a, b, d, p);
  dist_helper(x, projectx, b, c, d, p);
  dist_helper(x, projectx, c, a, d, p);
}

void dist_helper(const Eigen::RowVector3d & x,
                 const Eigen::RowVector3d & px,
                 const Eigen::RowVector3d & a,
                 const Eigen::RowVector3d & b,
                 double & d,
                 Eigen::RowVector3d & p) {
  Eigen::RowVector3d pa = a + ((px-a).dot(b-a)) / ((b-a).dot(b-a)) * (b-a);
  double distance;
  if (abs((pa-a).norm() + (a-b).norm() - (pa-b).norm()) < 1e-10) {
    distance = (x-a).norm();
    p = a;
  }
  else if (abs((pa-b).norm() + (b-a).norm() - (pa-a).norm()) <1e-10) {
    distance = (x-b).norm();
    p = b;
  }
  else {
    distance = (x-pa).norm();
    p = pa;
  }

  if (d > distance) {
    d = distance;
  }
}
