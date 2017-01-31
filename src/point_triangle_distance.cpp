#include "point_triangle_distance.h"
#include <igl/barycentric_coordinates.h>
#include <math.h>

double clamp(double num) {
  if (num < 0) {
    return 0;
  } else if (num > 1) {
    return 1;
  } else {
    return num;
  }
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  const Eigen::RowVector3d & n,
  double & d,
  Eigen::RowVector3d & p)
{
  double t;
  
  // Find proj of x onto plane of triangle ABC
  Eigen::RowVector3d proj;
  t = (n.dot(a) - n.dot(x)) / n.dot(n);
  proj = x + t*n;
  
  // Find barycentric point on triangle and clamp if necessary
  Eigen::RowVector3d bary;
  igl::barycentric_coordinates(proj, a, b, c, bary);
  double num;
  if (bary(0) < 0) {
    num = (proj-b).dot(c-b) / (c-b).dot(c-b);
    num = clamp(num);
    bary << 0.0, 1.0 - num, num;
  } else if (bary(1) < 0) {
    num = (proj-c).dot(a-c) / (a-c).dot(a-c);
    num = clamp(num);
    bary << num, 0.0, 1.0 - num;
  } else if (bary(2) < 0) {
    num = (proj-a).dot(b-a) / (b-a).dot(b-a);
    num = clamp(num);
    bary << 1.0 - num, num, 0.0;
  }
  
  // Translate barycentric point to cartesian coordinates, and this is
  // the closest point on the triangle
  p = (bary(0) * a) + (bary(1) * b) + (bary(2) * c);
  
  // Find distance
  d = sqrt( pow(x(0) - p(0), 2) + pow(x(1) - p(1), 2) + pow(x(2) - p(2), 2) );
}
