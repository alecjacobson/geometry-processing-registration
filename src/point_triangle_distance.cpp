#include "point_triangle_distance.h"
#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  //find the plane spanned by (b-a) and (c-a)
  Eigen::RowVector3d n = (b-a).cross(c-a);
  n.normalize();
  Eigen::RowVector3d pt = (x-c);
  double dist = pt.dot(n);
  Eigen::RowVector3d closest_pt = x - (dist * n);
  // check if the point is in the triangle
    if(in_triangle(closest_pt, a, b, c)){
      d = (closest_pt - x).norm();
      p = closest_pt;
    }else{
      std::vector<Eigen::RowVector3d> candidate_pts;
      // add vertices to candidate_pts
      candidate_pts.push_back(a); candidate_pts.push_back(b); candidate_pts.push_back(c);
      // add closest points on triangle edges to candidate_pts

      // LINE AB
      Eigen::RowVector3d u = a - b;
      u /= u.norm();
      Eigen::RowVector3d v = a - x;
      candidate_pts.push_back(u.dot(v) * u);
      // LINE BC
      u = b - c;
      u /= u.norm();
      v = b - x;
      candidate_pts.push_back(u.dot(v) * u);
      // LINE AC
      u = a - c;
      u /= u.norm();
      v = c - x;
      candidate_pts.push_back(u.dot(v) * u);

      // find min distance and point among candidate_pts
      d = (a-x).norm();
      p = a;
      for(std::vector<Eigen::RowVector3d>::iterator it = candidate_pts.begin() ; it != candidate_pts.end(); ++it){
        if((*it-x).norm() < d){
          d = (*it-x).norm();
          p = *it;
        }
      }
    }
}

// Reference: http://blackpawn.com/texts/pointinpoly/
bool same_side(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c)
{
  Eigen::RowVector3d cross_x_bc = (c-b).cross(x-b);
  Eigen::RowVector3d cross_a_bc = (c-b).cross(a-b);
  return cross_x_bc.dot(cross_a_bc) >= 0;
}

bool in_triangle(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c)
{
  return same_side(x, a, b,c) && same_side(x, b, a, c) && same_side(x, c, a,b);
}
