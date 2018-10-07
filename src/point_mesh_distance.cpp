#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

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
  N = Eigen::MatrixXd::Zero(X.rows(), X.cols());
  D.resize(X.rows());

  // compute all Y face surface normals
  Eigen::MatrixXd N_all;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,0,0), N_all);

  // brute-force search
  int ii_min;
  double d, d_min;
  Eigen::RowVector3d p, p_min;
  for(int i = 0; i < X.rows(); ++i) {
    // reset d_min to large value
    d_min = 1e9;

    // search for closest face and point in Y for point Xi
    for (int ii = 0; ii < FY.rows(); ++ii) {
      const Eigen::RowVector3i& F = FY.row(ii); 
      point_triangle_distance(X.row(i), 
          VY.row(F(0)), VY.row(F(1)), VY.row(F(2)), 
          d, p);
      if (d < d_min) {
        d_min = d;
        ii_min = ii;
        p_min = p;
      }
    } // end loop ii

    D(i) = d_min;
    P.row(i) = p_min;
    N.row(i) = N_all.row(ii_min);
  } // end loop i

}
