#ifndef POINT_TRIANGLE_DISTANCE_H
#define POINT_TRIANGLE_DISTANCE_H
#include <Eigen/Core>
// Compute the distance `d` between a given point `x` and the closest point `p` on
// a given triangle with corners `a`, `b`, and `c`.
//
// Inputs:
//   x  3d query point
//   a  3d position of first triangle corner
//   b  3d position of second triangle corner
//   c  3d position of third triangle corner
// Outputs:
//   d  distance from x to closest point on triangle abc
//   p  3d position of closest point 
void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p);
#endif
