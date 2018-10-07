#include "point_triangle_distance.h"
#include <Eigen/Geometry>
#include <algorithm>

using namespace Eigen;
using namespace std;

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  RowVector3d normal = (b - a).cross(c - b);
  normal.normalize();

  double area_ABC = normal.dot((b - a).cross(c - b));
  double area_PBC = normal.dot((b - p).cross(c - p));
  double area_PCA = normal.dot((c - p).cross(a - p));

  double alpha = area_PBC * 1.0 / area_ABC;
  double beta = area_PCA * 1.0 / area_ABC;
  double gamma = 1.0 - alpha - beta;

  RowVector3d p0 = x - x.dot(normal) * normal;

  // reference: https://stackoverflow.com/questions/14467296/barycentric-coordinate-clamping-on-3d-triangle 
  Eigen::RowVector3d pc;

  if (gamma < 0) {
      double t = (p0 - b).dot(a - b) / (a - b).dot(a - b);
      t = max(0.0, min(t, 1.0));
      pc = RowVector3d(t, 1.0 - t, 0.0);
  }
  else if (beta < 0) {
      double t = (p0 - c).dot(c - a) / (c - a).dot(c - a);
      t = max(0.0, min(t, 1.0));
      pc = RowVector3d(1.0 - t, 0.0, t);
  }
  else if (alpha < 0) {
      double t = (p0 - c).dot(b - c) / (b - c).dot(b - c);
      t = max(0.0, min(t, 1.0));
      pc = RowVector3d(0.0, t, 1.0 - t);
  }
  else {
      pc = RowVector3d(alpha, beta, gamma);
  }

  p = pc[0] * a + pc[1] * b + pc[2] * c;
  
  d = (x - p).norm();
}