#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

#include <iostream>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code

  igl::per_face_normals(VY, FY, N);
  P.resizeLike(X);
  D.resize(X.rows());
  
  for(int i = 0; i < X.rows(); i++) {
    double min_dist = 1e300;
    Eigen::RowVector3d closest_p;

    for(int face_i = 0; face_i < FY.rows(); face_i++) {
      double this_dist;
      Eigen::RowVector3d this_p;
      point_triangle_distance(
        X.row(i),
        VY.row(FY(face_i, 0)),
        VY.row(FY(face_i, 1)),
        VY.row(FY(face_i, 2)),
        this_dist,
        this_p
      );

      if(this_dist < min_dist) {
        closest_p = this_p;
        min_dist = this_dist;
      }
    }
    // std::cout << min_dist << std::endl;
    P.row(i) = closest_p;
    D[i] = min_dist;
  }
}
