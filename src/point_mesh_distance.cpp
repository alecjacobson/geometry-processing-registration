#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include "igl/per_face_normals.h"

void point_mesh_distance(
    const Eigen::MatrixXd &X,
    const Eigen::MatrixXd &VY,
    const Eigen::MatrixXi &FY,
    Eigen::VectorXd &D,
    Eigen::MatrixXd &P,
    Eigen::MatrixXd &N)
{
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(), X.cols());
  D.resize(X.rows());

  for (int i = 0; i < X.rows(); i++)
  {

    double minD = std::numeric_limits<double>::max();
    Eigen::RowVector3d minP;
    for (int j = 0; j < FY.rows(); j++)
    {
      double d;
      Eigen::RowVector3d p;
      point_triangle_distance(X.row(i),VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), d, p);
      if (minD < d)
      {
        minD = d;
        minP = p;
      }
    }

    D(i) = minD;
    P.row(i) = minP;
  }

  igl::per_face_normals(VY,FY,N);
}
