#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
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
  // P.resizeLike(X);
  // N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  // for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  // D = (X-P).rowwise().norm();

  // My code
  // compute face normals
  Eigen::MatrixXd FN;
  igl::per_face_normals(VY,FY,Eigen::Vector3d(1,1,1).normalized(),FN);

  D.resize(X.rows());
  P.resize(X.rows(), 3);
  N.resize(X.rows(), 3);

  Eigen::RowVector3d Xpt, a, b, c, p;
  double dist;
  double minDist;
  int minIdx;
  double pmin_x, pmin_y, pmin_z;
  for (int ii = 0; ii < X.rows(); ii++){ // loop over points X
    minDist = std::numeric_limits<double>::max();

    for (int jj = 0; jj < FY.rows(); jj++){ // loop over triangles
      Xpt = X.row(ii);
      a = VY.row(FY(jj,0));
      b = VY.row(FY(jj,1));
      c = VY.row(FY(jj,2));
      point_triangle_distance(Xpt, a, b, c, dist, p);

      if (dist < minDist){
        minDist = dist;
        minIdx = jj;
        pmin_x = p(0);
        pmin_y = p(1);
        pmin_z = p(2);
      }
    }
    D(ii) = minDist;
    P.row(ii) << pmin_x, pmin_y, pmin_z;
    N.row(ii) = FN.row(minIdx);
  }
}
