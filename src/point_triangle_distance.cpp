#include "point_triangle_distance.h"
#include <Eigen/Geometry> 
#include <algorithm>

// returns true if p1 and p2 are on the same side of the line ab
// http://blackpawn.com/texts/pointinpoly/
bool same_side(
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & p1,
  const Eigen::RowVector3d & p2
)
{
  Eigen::Vector3d c1, c2;
  c1 = (b - a).cross(p1 - a);
  c2 = (b - a).cross(p2 - a);
  if (c1.dot(c2) >= 0) {
    return true;
  }
  return false;
}

// returns (via d and p) closest point and its distance from x to the line segment ab
// https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
void closest_point_on_line(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  double & d,
  Eigen::RowVector3d & p
)
{
  double t = std::max(0.0, std::min(1.0, (x - a).dot(b - a) / (b - a).squaredNorm()));
  Eigen::RowVector3d projection = a + t * (b - a);
  p = projection;
  d = (x - projection).norm();
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // project point onto plane
  Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(a, b, c);
  Eigen::RowVector3d projection = plane.projection(x);

  // if the projection is inside the triangle, return it
  if (same_side(a, b, c, x) && same_side(b, c, a, x) && same_side(c, a, b, x)) {
    d = (x - projection).norm();
    p = projection;
  }
  
  // otherwise, return the distance to the closest side (edge)
  else {
    Eigen::RowVector3d currentP, closestP;
    double closestD = std::numeric_limits<double>::max();
    double currentD;
    Eigen::RowVector3d sides [3] = {a, b, c};

    // loop over sides to find min distance
    for (int i = 0; i < 3; i++) {
      closest_point_on_line(x, sides[i], sides[(i+1)%3], currentD, currentP);
      if (currentD < closestD) {
        closestD = currentD;
        closestP = currentP;
      }
    } 
    d = closestD;
    p = closestP;
  }

}
