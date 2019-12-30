#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  Eigen::RowVector3d center = (a + b + c) / 3;

  d = (center - x).norm();
  p = c;
}
