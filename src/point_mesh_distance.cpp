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
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());

  Eigen::MatrixXd normals;
  igl::per_face_normals(VY,FY,Eigen::Vector3d(1,1,1).normalized(),normals);

  for(int i = 0;i<X.rows();i++) {
    Eigen::RowVector3d proj_y;
    double distance = std::numeric_limits<double>::max();;
    int index_tr = 0;

    for(int j = 0;j<FY.rows();j++){
      Eigen::RowVector3d temp_proj_y;
      double temp_distance = 1.79769e+308;

      point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), temp_distance, temp_proj_y);

      if (temp_distance < distance){
        distance = temp_distance;
        proj_y = temp_proj_y;
        index_tr = j;
      }
    }

    P.row(i) = proj_y;
    D(i) = distance;
    N.row(i) = normals.row(index_tr);
  }
}
