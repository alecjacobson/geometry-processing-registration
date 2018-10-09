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
    P.resizeLike(X);
    N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    D.resize(X.rows());
    Eigen::MatrixXd normals;
    igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), normals);
    for(int i = 0;i<X.rows();i++) {
        double minD = 100000000;
        double d_;
        int tri;
        Eigen::RowVector3d p_, minP;
        for (int j = 0; j < FY.rows(); j++) {
            point_triangle_distance(X.row(i), VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), d_, p_);
            if (d_ < minD) {
                minD = d_;
                minP = p_;
                tri = j;
            }
        }
        P.row(i) = minP;
        N.row(i) = normals.row(tri);
        D(i) = minD;
    }
}
