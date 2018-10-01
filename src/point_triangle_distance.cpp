#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // NOTE: Can be optimized if needed, doing lazily

  // Seems like the proper algorithm using barycentric
  // coordinates would need to project to two edges anyway
  // (for obtuse triangles)

  d = std::numeric_limits<double>::max();

  Eigen::RowVector3d ba = b - a;
  Eigen::RowVector3d ca = c - a;
  Eigen::RowVector3d xa = x - a;
  Eigen::RowVector3d n = ba.cross(ca);
  n.normalize();

  // find point on plane
  double t = n.dot(xa);
  Eigen::RowVector3d pop = x + t*n;

  // find barycentric coordinates
  double d00 = ba.dot(ba);
  double d01 = ba.dot(ca);
  double d11 = ca.dot(ca);
  double d20 = xa.dot(ba);
  double d21 = xa.dot(ca);

  double denom = d00 * d11 - d01 * d01;

  double beta = (d11 * d20 - d01 * d21) / denom;
  double gamma = (d00 * d21 - d01 * d20) / denom;
  double alpha = 1 - beta - gamma; 

  if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && gamma > 0 && gamma < 1){
    p = alpha * a + beta * b + gamma * c;
    d = (p - x).norm();
    return;
  }

  // check three vertices
  double tmp = (x-a).norm();
  if (d > tmp){
    d = tmp;
    p = a;
  }

  tmp = (x-b).norm();
  if (d > tmp){
    d = tmp;
    p = b;
  }

  tmp = (x-c).norm();
  if (d > tmp){
    d = tmp;
    p = c;
  }

  // check three edges
  // ab
  t = (b - a).dot(x - a)/(b-a).dot(b-a);
  if (t > 0 && t < 1){
    Eigen::RowVector3d tmp_p = a + t*(b-a);
    tmp = (tmp_p - x).norm();
    if (d > tmp){
      p = tmp_p;
      d = tmp;
    }
  }

  // bc
  t = (c - b).dot(x - b)/(c-b).dot(c-b);
  if (t > 0 && t < 1){
    Eigen::RowVector3d tmp_p = b + t*(c-b);
    tmp = (tmp_p - x).norm();
    if (d > tmp){
      p = tmp_p;
      d = tmp;
    }
  }

  // ca
  t = (a - c).dot(x - c)/(a-c).dot(a-c);
  if (t > 0 && t < 1){
    Eigen::RowVector3d tmp_p = c + t*(a-c);
    tmp = (tmp_p - x).norm();
    if (d > tmp){
      p = tmp_p;
      d = tmp;
    }
  }
}