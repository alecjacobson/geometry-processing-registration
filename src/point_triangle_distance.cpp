#include "point_triangle_distance.h"
#include <limits>
#include <Eigen/Geometry>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  d = std::numeric_limits<double>::infinity();
  double delta = 1e-6;

  auto cross = [](Eigen::RowVector3d a, Eigen::RowVector3d b){
    return a.cross(b);
  };

  Eigen::RowVector3d n = cross(b.transpose() - a.transpose(), c.transpose() - a.transpose());
  n.normalize();

  double t = (abs((x - a).dot(n)) > delta) ? (a.dot(n)) / (x.dot(n)) : 0.0;
  Eigen::RowVector3d q = x + t * n;

  auto inTriangle = [&a, &b, &c, &cross, &delta](Eigen::RowVector3d q){
      // Check point in triangle
      double area = cross(a - b, a - c).norm() / 2;
      double pct0 = cross(q - b, q - c).norm() / 2;
      double pct1 = cross(q - a, q - b).norm() / 2;
      double pct2 = cross(q - c, q - a).norm() / 2;

      return abs(area - pct0 - pct1 - pct2) < delta;
  };

  if (inTriangle(q)) {
      d = t;
      p = q;
      return;
  }

  // Not in triangle
  auto checkPointDist = [&d, &p, &x](Eigen::RowVector3d q){
    Eigen::RowVector3d v = x - q;
    double n = v.norm();
    if (n < d) {
      d = n;
      p = q;
    }
  };
  checkPointDist(a);
  checkPointDist(b);
  checkPointDist(c);

  auto checkEdgeDist = [&t, &d, &p, &inTriangle](Eigen::RowVector3d pt, Eigen::RowVector3d v1, Eigen::RowVector3d v2){
    Eigen::RowVector3d edge = (v1 - v2);
    Eigen::RowVector3d proj = ((pt - v2).dot(edge) / edge.squaredNorm()) * edge;
    Eigen::RowVector3d nproj = pt - v2  - proj;

    double dist = sqrt(t * t + nproj.squaredNorm());
    if (dist < d && inTriangle(v2 + proj)) {
        d = dist;
        p = v2 + proj;
    }
  };
  checkEdgeDist(q, a, b);
  checkEdgeDist(q, a, c);
  checkEdgeDist(q, b, c);
}
