#include "point_triangle_distance.h"
#include <limits>

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
  Eigen::RowVector3d ba = b-a;
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
  if (d > (x - a).norm()) {
  	d = (x - a).norm();
  	p = a;
  }
  if (d > (x - b).norm()) {
  	d = (x - b).norm();
  	p = b;
  }
  if (d > (x - c).norm()) {
  	d = (x - c).norm();
  	p = c;
  }
}
