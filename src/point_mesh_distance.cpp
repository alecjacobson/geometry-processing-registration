#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

using namespace Eigen;
using namespace std;

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());

  MatrixXd faceNormals;
  igl::per_face_normals(VY, FY, Vector3d(1,1,1).normalized(), faceNormals);

  double min_distance, temp_distance;
  RowVector3d closest_point, temp_point;
  int closest_index = 0;
  Vector3d point;

  for (int i = 0; i < X.rows(); i++) {
    min_distance = -1;
    point = X.row(i);

    for (int j = 0; j < FY.rows(); j++) {
      point_triangle_distance(point, VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), temp_distance, temp_point);
      if (temp_distance < min_distance || min_distance < 0) {
        min_distance = temp_distance;
        closest_index = j;
        closest_point = temp_point;
      }
    }

    D(i) = min_distance;
    P.row(i) = closest_point;
    N.row(i) = faceNormals.row(closest_index);
  }
}