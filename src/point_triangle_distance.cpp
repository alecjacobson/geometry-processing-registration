#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code

  // algorithm from: 
  // Distance Between Point and Triangle in 3D, David Eberly
  // https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf

  Eigen::RowVector3d e0 = b - a;
  Eigen::RowVector3d e1 = c - a;
  Eigen::RowVector3d ax = a - x;

  double q_a = e0.dot(e0);
  double q_b = e0.dot(e1);
  double q_c = e1.dot(e1);
  double q_d = e0.dot(ax);
  double q_e = e1.dot(ax);
  double det = q_a*q_c - q_b*q_b;
  double s = q_b*q_e - q_c*q_d;
  double t = q_b*q_d - q_a*q_e;

  if (s + t <= det) {
    if (s < 0) {
      if (t < 0) {
        // region 4
        if (q_d < 0) {
          s = std::max(0.0, std::min(-q_d/q_a, 1.0));
          t = 0;
        }
        else {
          s = 0;
          t = std::max(0.0, std::min(-q_e/q_c, 1.0));
        }
      }
      else {
        // region 3
        s = 0;
        t = std::max(0.0, std::min(-q_e/q_c, 1.0));
      }
    }
    else if (t < 0) {
      // region 5
      s = std::max(0.0, std::min(-q_d/q_a, 1.0));
      t = 0;
    }
    else {
      // region 0
      s /= det;
      t /= det;
    }
  }
  else {
    if (s < 0) {
      // region 2
      double temp0 = q_b + q_d;
      double temp1 = q_c + q_e;
      if (temp1 > temp0) {
        double numer = temp1 - temp0;
        double denom = q_a - 2*q_b + q_c;
        s = std::max(0.0, std::min(numer/denom, 1.0));
        t = 1 - s;
      }
      else {
        t = std::max(0.0, std::min(-q_e/q_c, 1.0));
        s = 0;
      }
    }
    else if (t < 0) {
      // region 6
      double temp0 = q_b + q_e;
      double temp1 = q_a + q_d;
      if (temp1 > temp0) {
        double numer = q_c + q_e - q_b - q_d;
        double denom = q_a - 2*q_b + q_c;
        s = std::max(0.0, std::min(numer/denom, 1.0));
        t = 1 - s;
      }
      else {
        s = std::max(0.0, std::min(-q_e/q_c, 1.0));
        t = 0;
      }
    }
    else {
      // region 1
      double numer = q_c + q_e - q_b - q_d;
      double denom = q_a - 2*q_b + q_c;
      s = std::max(0.0, std::min(numer/denom, 1.0));
      t = 1 - s;
    }
  }

  p = a + s*e0 + t*e1;
  d = (x - p).norm();
}
