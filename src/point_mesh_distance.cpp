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
  D.resize(X.rows());
  P.resizeLike(X);
  N.resizeLike(X);

  Eigen::MatrixXd NY;
  NY.resizeLike(FY);
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), NY);

  for (int i = 0; i < X.rows(); i++) {
    Eigen::RowVector3d x = X.row(i);

    double d = std::numeric_limits<double>::infinity();
    Eigen::RowVector3d p;
    Eigen::RowVector3d n;
    // Iterative method
    for (int j = 0; j < FY.rows(); j++) {
      Eigen::RowVector3d a = VY.row(FY(j, 0));
      Eigen::RowVector3d b = VY.row(FY(j, 1));
      Eigen::RowVector3d c = VY.row(FY(j, 2));

      double d_tmp;
      Eigen::RowVector3d p_tmp;
      point_triangle_distance(x, a, b, c, d_tmp, p_tmp);
      if (d_tmp < d) {
        d = d_tmp;
        p = p_tmp;
        n = NY.row(j);
      }
    }

    D(i) = d;
    P.row(i) = p;
    N.row(i) = n;
  }

}
