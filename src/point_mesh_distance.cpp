#include "point_mesh_distance.h"
#include <limits>
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
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  double d;
  Eigen::RowVector3d p;
  Eigen::MatrixXd pfn;
  igl::per_face_normals(VY,FY,Eigen::Vector3d(1,1,1).normalized(),pfn);
  for(int i = 0;i<X.rows();i++) {
    D(i) = std::numeric_limits<double>::max();
    for(int j = 0;j<FY.rows();j++) {
      point_triangle_distance(X, VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), 
        d, p);
      if (D(i) > d) {
        D(i) = d;
        P.row(i) = p;
        N.row(i) = pfn.row(j);
      }
    }
  }
}
