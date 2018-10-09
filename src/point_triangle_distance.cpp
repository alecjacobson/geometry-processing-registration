#include "point_triangle_distance.h"
#include <Eigen/Geometry>

void point_on_line(const Eigen::RowVector3d &x,
                   const Eigen::RowVector3d &a,
                   const Eigen::RowVector3d &b,
                   const Eigen::RowVector3d &projectP,
                   double &d,
                   Eigen::RowVector3d &p)
{
  Eigen::RowVector3d nab = ((b - projectP).cross(a - projectP)).cross(b - a);
  nab.normalize();
  double d_ab = abs((a - projectP).dot(nab));
  Eigen::RowVector3d pab = projectP + d_ab * nab;
  double t = (pab - a).dot(b - a) / (b - a).dot(b - a);
  if (t < 0)
  {
    p = a;
    d = (x - p).norm();
  }
  if (0 < t < 1)
  {
    p = pab;
    d = (x - p).norm();
  }
  if (t > 1)
  {
    p = b;
    d = (x - p).norm();
  }
}

void point_triangle_distance(
    const Eigen::RowVector3d &x,
    const Eigen::RowVector3d &a,
    const Eigen::RowVector3d &b,
    const Eigen::RowVector3d &c,
    double &d,
    Eigen::RowVector3d &p)
{
  Eigen::RowVector3d v1 = b - a;
  Eigen::RowVector3d v2 = -a + c;
  Eigen::RowVector3d n = v1.cross(v2);
  n.normalize();

  double distance = abs((x - a).dot(n));
  Eigen::RowVector3d projectP = x - distance * n;
  Eigen::RowVector3d v3 = projectP - a;
  Eigen::RowVector3d v4, v5, v6;

  // based on https://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
  double u = (v2.dot(v2) * v3.dot(v1) - v1.dot(v2) * v3.dot(v2)) * 1.0 /
             (v2.dot(v2) * v1.dot(v1) - v1.dot(v2) * v1.dot(v2));

  double v = (v1.dot(v1) * v3.dot(v2) - v1.dot(v2) * v3.dot(v1)) * 1.0 /
             (v2.dot(v2) * v1.dot(v1) - v1.dot(v2) * v1.dot(v2));

  if (u >= 0 && v >= 0 && (u + v <= 1))
  { //inside
    p = a + u * v1 + v * v2;
    d = distance;
  }
  else if (u >= 0 and v < 0)
  {
    // On ab
    point_on_line(x, a, b, projectP, d, p);
  }
  else if (u < 0 && v >= 0)
  {
    //On ac
    point_on_line(x, a, c, projectP, d, p);
  }
  else if (u >= 0 && v >= 0 && u + v > 1)
  {
    //On bc
    point_on_line(x, b, c, projectP, d, p);
  }
  else
  { //Outside
    p = a;
    d = (x - p).norm();
  }
}
