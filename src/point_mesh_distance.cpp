#include "point_mesh_distance.h"
#include <igl/per_face_normals.h>
#include "point_triangle_distance.h"
#include <limits>
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
  //N.resize(X.rows(),3);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  //for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  //D = (X-P).rowwise().norm();
  //D.resize(X.rows(), 1);
  Eigen::MatrixXi triangle_F;
  triangle_F.resize(X.rows(),3);

  for (int i=0; i<X.rows(); i++) {
    double idist = std::numeric_limits<double>::max();
    Eigen::RowVector3d x = X.row(i);
    Eigen::RowVector3d ip;

    for (int j=0; j<FY.rows(); j++) {
      double cur_dist;
      Eigen::RowVector3d cur_p,a,b,c;
      a = VY.row(FY(j,0));
      b = VY.row(FY(j,1));
      c = VY.row(FY(j,2));
      point_triangle_distance(x,a,b,c,cur_dist,cur_p);
      if (cur_dist < idist) {
        idist = cur_dist;
        ip = cur_p;
        triangle_F.row(i) = FY.row(j);
      }
    }

    P.row(i) = ip;
    D(i) = idist;
  }
  igl::per_face_normals(VY,triangle_F,N);
}
