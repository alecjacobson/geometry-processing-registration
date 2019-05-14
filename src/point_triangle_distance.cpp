#include "point_triangle_distance.h"
#include <Eigen/Geometry>
#include <algorithm>

double clamp(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Project onto plane
  // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d
  d = 0;
  p = a;

  Eigen::RowVector3d orig = a;
  Eigen::RowVector3d to_orig = x - orig;
  Eigen::RowVector3d v0 = b - a, v1 = c - a, v2 = x - a;
  Eigen::RowVector3d n = v0.cross(v1);
  n = n.normalized();

  double dist = to_orig.dot(n);

  Eigen::RowVector3d proj_p = x - dist * n;

  // Compute barycentric coords
  // http://gamedev.stackexchange.com/a/23745
  double d00 = v0.dot(v0);
  double d01 = v0.dot(v1);
  double d11 = v1.dot(v1);
  double d20 = v2.dot(v0);
  double d21 = v2.dot(v1);
  double denom = d00 * d11 - d01 * d01;
  double v = (d11 * d20 - d01 * d21) / denom;
  double w = (d00 * d21 - d01 * d20) / denom;
  double u = 1.0f - v - w;

  // Get closest point on tri
  //https://stackoverflow.com/questions/14467296/barycentric-coordinate-clamping-on-3d-triangle 
  Eigen::RowVector3d p_closest_bry;
  if ( u < 0)
  {
      double t = (proj_p-b).dot(c-b)/(c-b).dot(c-b);
      t = clamp(t, 0.0, 1.0);
      p_closest_bry = Eigen::RowVector3d( 0.0, 1.0-t, t );
  }
  else if ( v < 0 )
  {
      double t = (proj_p-c).dot(a-c)/(a-c).dot(a-c);
      t = clamp(t, 0.0, 1.0);
      p_closest_bry = Eigen::RowVector3d( t, 0.0, 1.0-t );
  }
  else if ( w < 0 )
  {
      double t = (proj_p-a).dot(b-a)/(b-a).dot(b-a);
      t = clamp(t, 0.0, 1.0);
      p_closest_bry = Eigen::RowVector3d( 1.0-t, t, 0.0 );
  }
  else
  {
      p_closest_bry = Eigen::RowVector3d( u, v, w );
  }

  p = p_closest_bry[0] * a + p_closest_bry[1] * b + p_closest_bry[2] * c;
  d = (x-p).norm();
}
