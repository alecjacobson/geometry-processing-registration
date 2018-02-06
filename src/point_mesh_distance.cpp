#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <limits>
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
  D.resize(X.rows(), 1);
  N.resize(X.rows(), 3);
  //N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  //for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  //D = (X-P).rowwise().norm();
  Eigen::MatrixXi temp_F;
  temp_F.resize(X.rows(),3);
  for (int i = 0; i< X.rows(); ++i){
    double min_d =  std::numeric_limits<double>::max();
    Eigen::RowVector3d min_p;
    int min_f;
    for (int j = 0; j < FY.rows(); ++j){
      double temp_d = 0;
      Eigen::RowVector3d temp_p;
      point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), temp_d, temp_p);
      if (temp_d < min_d){
        min_d = temp_d;
        min_p = temp_p;
        min_f = j;
      }
    }
    D(i,0) = min_d;
    P.row(i) = min_p;
    temp_F.row(i) = FY.row(min_f);
  }
  igl::per_face_normals(VY,temp_F,Eigen::Vector3d(1,1,1).normalized(),N);  
}
