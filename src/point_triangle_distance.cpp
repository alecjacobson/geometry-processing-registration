#include "point_triangle_distance.h"
#include <iostream>
#include <Eigen/Dense>
#include <bitset>

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
  p = x;

  auto u = b-a;
  auto v = c-a;

  Eigen::RowVector3d n = u.cross(v).normalized();
  Eigen::RowVector3d xp = x - a;
  xp = xp - xp.dot(n) * n;

  std::bitset<3> inside;

  auto hpInside = [&](//half plane inside check
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c
  ) {
      auto u = (b-a).normalized().eval();
      auto v = c-a;
      auto n = (v - u.dot(v) * u).normalized().eval();
      double d = (xp-a).dot(n);
      return d >= 0;
  };

 inside[0] = hpInside(b,c,a);
 inside[1] = hpInside(c,a,b);
 inside[2] = hpInside(a,b,c);

 auto pplane = [&](//edge projection, computes redundant stuff
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b
  ) {
     auto u = p - a;
     auto v = (b - a).normalized().eval();
     return p - u.dot(v) * v;
 };

 if(inside[0] && inside[1] && inside[2]) {
     p = xp;
 } else if(inside[0] && inside[1]) {//outside edge ab
     p = pplane(a,b);
 } else if (inside[1] && inside[2]) {//outsdie edge bc
     p = pplane(b,c);
 } else if (inside[2] && inside[0]) {//outsdie edge ca
     p = pplane(c,a);
 } else if(inside[0]) {//a
     p = a;
 } else if(inside[1]) {//b
     p = b;
 } else if(inside[2]) {//c
     p = c;
 }

  
  d = (x-p).norm();
}
