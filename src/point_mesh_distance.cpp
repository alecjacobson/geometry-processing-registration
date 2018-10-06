#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  int n = X.rows();
  int m = FY.rows();
  D.resize(n);
  P.resize(n, 3);
  N.resize(n, 3);
  
  for (int i = 0; i < n; i++) {
    Eigen::RowVector3d x = X.row(i);
    double closest_d = INT_MAX;
    Eigen::RowVector3d closest_p;
    Eigen::RowVector3d closest_p_n;
    for (int j = 0; j < m; j++) {
      double d;
      Eigen::RowVector3d p;
      Eigen::Vector3d a = VY.row(FY(j, 0));
      Eigen::Vector3d b = VY.row(FY(j, 1));
      Eigen::Vector3d c = VY.row(FY(j, 2));
      point_triangle_distance(x, a, b, c, d, p);
      if (d < closest_d) {
        closest_d = d;
        closest_p = p;
        closest_p_n = (c-a).cross(c-b);
        closest_p_n /= closest_p_n.norm();
      }
    }
    D(i) = closest_d;
    P.row(i) = closest_p;
    N.row(i) = closest_p_n;
  }
}
