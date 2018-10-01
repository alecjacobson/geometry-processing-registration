#include "point_mesh_distance.h"
#include <iostream>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  P.resizeLike(X);
  D = Eigen::VectorXd(X.rows());
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());

  // Compute face normals
  Eigen::MatrixXd FN;
  igl::per_face_normals(VY,FY,Eigen::Vector3d(1,1,1).normalized(),FN);  
  
  for(int i = 0;i<X.rows();i++){
    double d = std::numeric_limits<double>::max(), tmp_d;
    Eigen::RowVector3d closest_point, tmp_point, b;
    int min_index = 0;

    for(int j = 0;j<FY.rows();j++){
      point_triangle_distance(
        X.row(i), 
        VY.row(FY(j, 0)),
        VY.row(FY(j, 1)),
        VY.row(FY(j, 2)),
        tmp_d, tmp_point
      );
      
      // std::cout << i << "," << j << "," << tmp_d << "," << d << "\n";

      if (tmp_d < d){
        d = tmp_d;
        closest_point = tmp_point;
        min_index = j;
      }
    }

    P.row(i) = closest_point;
    D(i) = d;
    N.row(i) = FN.row(min_index);

    // std::cout << min_index << "," << d << "\n";
  }
  
  D = (X-P).rowwise().norm();
}