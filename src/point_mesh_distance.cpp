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
  // Replace with your code
  // P.resizeLike(X);
  // N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  // for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  // D = (X-P).rowwise().norm();

  P.resizeLike(X);
  D.resize(X.rows());
  N.resizeLike(X);

  for (int i=0; i<X.rows(); i++) {

    double least_d = 0;
    Eigen::RowVector3d least_p;
    int least_faceidx = 0;

    for (int j=0; j<FY.rows(); j++) {

      double d_tmp;
      Eigen::RowVector3d p_tmp;

      point_triangle_distance(X.row(i).transpose(), 
                              VY.row(FY(j, 0)).transpose(),
                              VY.row(FY(j, 1)).transpose(),
                              VY.row(FY(j, 2)).transpose(),
                              d_tmp,
                              p_tmp);

      if (j==0) {
        least_d = d_tmp;
        least_p(0) = p_tmp(0); least_p(1) = p_tmp(1); least_p(2) = p_tmp(2); 
        least_faceidx = j;
      } else {
        if (d_tmp < least_d) {
          least_d = d_tmp;
          least_p(0) = p_tmp(0); least_p(1) = p_tmp(1); least_p(2) = p_tmp(2); 
          least_faceidx = j;
        }
      }


    }

    D(i) = least_d;
    P(i, 0) = least_p(0); P(i, 1) = least_p(1); P(i, 2) = least_p(2);
    
    Eigen::Vector3d LeastTriangleNormal;
    // Build V and F based on given coordinates
    Eigen::MatrixXd V_dummy(3,3);
    V_dummy.row(0) = VY.row(FY(least_faceidx, 0));
    V_dummy.row(1) = VY.row(FY(least_faceidx, 1));
    V_dummy.row(2) = VY.row(FY(least_faceidx, 2));
   
    Eigen::MatrixXd F_dummy(1,3);
    F_dummy << 0, 1, 2;

    Eigen::MatrixXd N_dummy(1,3);
    igl::per_face_normals(V_dummy, F_dummy, N_dummy);

    LeastTriangleNormal = N_dummy.row(0).normalized();

    N(i, 0) = LeastTriangleNormal(0); N(i, 1) = LeastTriangleNormal(1); N(i, 2) = LeastTriangleNormal(2);

  }

}
