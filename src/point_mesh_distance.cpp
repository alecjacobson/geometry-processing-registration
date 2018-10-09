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

  // Since assume all points in P is inside the face, could directly
  // compute all normals:
  Eigen::MatrixXd All_normal;
  igl::per_face_normals(VY, FY, 
          Eigen::Vector3d(1,1,1).normalized(), All_normal);

  // Compute closest points
  for (int i = 0; i < X.rows(); i ++) {
    Eigen::RowVector3d x = X.row(i);
    double min = std::numeric_limits<double>::max();
    Eigen::RowVector3d projection;
    Eigen::RowVector3d normal;

    // Loop over faces to find the closest point
    for(int j = 0; j < FY.rows(); j ++) {
      Eigen::RowVector3d a = VY.row(FY(j, 0));
      Eigen::RowVector3d b = VY.row(FY(j, 1));
      Eigen::RowVector3d c = VY.row(FY(j, 2));
      double cur_min;
      Eigen::RowVector3d cur_p;
      point_triangle_distance(x, a, b, c, cur_min, cur_p);
      if (min > cur_min) {
        min = cur_min;
        projection = cur_p;
        normal = All_normal.row(j);
      }
    }
    // Able to add to the result
    D(i) = min;
    P.row(i) = projection;
    N.row(i) = normal;
  }
}
