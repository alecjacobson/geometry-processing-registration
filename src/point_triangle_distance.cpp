#include "point_triangle_distance.h"
#include <Eigen/Geometry>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // obtain the normal to abc
  Eigen::RowVector3d n = (c-a).cross(c-b);
  n /= n.norm();
  
  // obtain a candidate p0 by projecting x onto triangle normal
  Eigen::RowVector3d p0 = x - (x-a).dot(n) * n;

  bool left_of_ab = (b-a).cross(p0-a).dot(n) >= 0;
  bool left_of_bc = (c-b).cross(p0-b).dot(n) >= 0;
  bool left_of_ca = (a-c).cross(p0-c).dot(n) >= 0;
  if (left_of_ab and left_of_bc and left_of_ca) {
    p = p0;  // p is inside the triangle
  }
  else {
    // obtain three more candidates by projecting p onto triangle edges
    Eigen::RowVector3d p_ab = a + (p0-a).dot(b-a) * (b-a);
    Eigen::RowVector3d p_bc = b + (p0-b).dot(c-b) * (c-b);
    Eigen::RowVector3d p_ca = c + (p0-c).dot(a-c) * (a-c);
    // set p according to region p0 is in
    if (left_of_ca && left_of_bc)
      p = p_ab;
    else if (left_of_ab && left_of_ca)
      p = p_bc;
    else if (left_of_bc && left_of_ab)
      p = p_ca;
    else if (!left_of_ca && !left_of_ab)
      p = a;
    else if (!left_of_ab && !left_of_bc)
      p = b;
    else if (!left_of_bc && !left_of_ca)
      p = c;
  }
  
  d = (x - p).norm();
}
