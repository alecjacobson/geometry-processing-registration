#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  Eigen::RowVector3d edge_ba = b - a;
  Eigen::RowVector3d edge_ca = c - a;
  Eigen::RowVector3d edge_xa = x - a;

  // Find the normal vector and projection
  Eigen::RowVector3d norm_vec = (edge_ba.cross(edge_ca)).normalized();
  Eigen::RowVector3d px = x - (norm_vec.dot(edge_xa))*norm_vec;

  // Use Barycentric algorithm to compute distance:
  // Idea inspired by https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
  Eigen::RowVector3d edge_pxa = px - a;
  double edge_ba_norm = edge_ba.dot(edge_ba);
  double edge_ca_norm = edge_ca.dot(edge_ca);
  double edge_ba_ca = edge_ba.dot(edge_ca);
  double edge_ba_pxa = edge_ba.dot(edge_pxa);
  double edge_ca_pxa = edge_ca.dot(edge_pxa);

  double denominator = edge_ba_norm*edge_ca_norm - edge_ba_ca*edge_ba_ca;

  double c2 = (edge_ca_norm*edge_ba_pxa - edge_ba_ca*edge_ca_pxa) / denominator;
  double c3 = (edge_ba_norm*edge_ca_pxa - edge_ba_ca*edge_ba_pxa) / denominator;
  double c1 = 1 - c2 - c3;

  // Handle if not inside the triangle by clamping these coordinates:
  c1 = c1 < 0 ? 0 : c1;
  c1 = c1 > 1 ? 1 : c1;
  c2 = c2 < 0 ? 0 : c2;
  c2 = c2 > 1 ? 1 : c2;
  c3 = 1 - c1 - c2;

  // Find value:
  p = c1 * a + c2 * b + c3 * c;
  d = (p - x).norm();
}
