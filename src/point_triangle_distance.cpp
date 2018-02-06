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
      // http://geomalgorithms.com/a02-_lines.html
      // LINE AB ... this should all be refactored daflhdsfa
      Eigen::RowVector3d v = b - a;
      Eigen::RowVector3d w = x - a;
      double c1 = w.dot(v);
      double c2 = v.dot(v);
      if(c1 > 0 && c2 > c1){
        double t = c1/c2;
        candidate_pts.push_back(a + t * v);
      }
      // LINE BC
      v = c - b;
      w = x - b;
      c1 = w.dot(v);
      c2 = v.dot(v);
      if(c1 > 0 && c2 > c1){
        double t = c1/c2;
        candidate_pts.push_back(b + t * v);
      }
      // LINE AC
      v = c - a;
      w = x - a;
      c1 = w.dot(v);
      c2 = v.dot(v);
      if(c1 > 0 && c2 > c1){
        double t = c1/c2;
        candidate_pts.push_back(a + t * v);
      }

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
