#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <Eigen/Dense>
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
  N.resizeLike(X);
  D.resize(X.rows());
  for(int i = 0;i<X.rows();i++) {
      double myd = std::numeric_limits<double>::max();
      Eigen::RowVector3d myp;
      int myt;

      Eigen::RowVector3d x = X.row(i);
      for(int t = 0; t < FY.rows(); ++t) {
          double d;
          Eigen::RowVector3d p;
          auto T = FY.row(t);
          point_triangle_distance(x,VY.row(T(0)),VY.row(T(1)),VY.row(T(2)),d,p);
          if(d < myd) {
              myd = d;
              myp = p;
              myt = t;
          }
      }
      D(i) = myd;
      P.row(i) = myp;
      auto T = FY.row(myt);
      Eigen::RowVector3d a = VY.row(T(1))- VY.row(T(0));
      Eigen::RowVector3d b = VY.row(T(2))- VY.row(T(0));
      N.row(i) = b.cross(a).normalized();
  }
}
