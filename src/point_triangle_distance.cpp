#include "point_triangle_distance.h"
#include <Eigen/Geometry>

bool point_in_triangle(
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  const Eigen::RowVector3d & p)
{
  Eigen::RowVector3d pa,pb,pc;
  pa = (p - a).normalized();
  pb = (p - b).normalized();
  pc = (p - c).normalized();
  double cos_theta = pa.dot(pb) + pa.dot(pc) + pb.dot(pc);
  return (cos_theta == 1.0);
}

void point_line_distance (
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  double & d,
  Eigen::RowVector3d & p
) 
{
  Eigen::RowVector3d ab,xa,n;
  ab = a - b;
  xa = x - a;
  n = ab.cross(xa);
  if ((n.norm() == 0) && (xa.norm() <= ab.norm())) { // x on ab
    p = x;
    d = 0;
    return;
  }

  double dist = xa.dot(n);
  Eigen::RowVector3d p_pontential = x - dist * n;

  if ((p_pontential - a).norm() > ab.norm()) { // p is outside ab
    p = b;
    d = (x - b).norm();
  }
  
  else if ((p_pontential - b).norm() > ab.norm())
  {
    p = a;
    d = (x - a).norm();
  }
  else {
    p = p_pontential;
    d = dist;
  }

}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  // d = 0;
  // p = a;
  Eigen::RowVector3d n = ((a-b).cross(a-c)).normalized();
  p = x - n.dot(x)*n;
  // p in triangle
  if (point_in_triangle(a,b,c,p)) {
    d = (x - p).norm();
  }
  else // p outside triangle
  {
    double ad,bd,cd;
    Eigen::RowVector3d ap,bp,cp;
    point_line_distance(x,a,b,ad,ap);
    point_line_distance(x,a,c,cd,cp);
    point_line_distance(x,b,c,bd,bp);
    d = ad;
    p = ap;
    if (cd > d) {
      d = cd;
      p = cp;
    }
    if (bd > d) {
      d = bd;
      p = bp;
    }
  }
}
