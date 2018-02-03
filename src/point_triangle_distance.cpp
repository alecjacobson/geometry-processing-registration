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
  d = 0;
  //Create the vectors of the triangle
    Eigen::RowVector3d u = a - b;
    Eigen::RowVector3d v = c - b;
    u = u / u.norm();
    v = v / v.norm();
    v = v - u.dot(v) * u;
    
    Eigen::RowVector3d projectedPoint;
    p = u.dot(x) * u + v.dot(x) * v;
    d = sqrt(p.dot(p) + x.dot(x) - 2* x.dot(p));
    
    
}
