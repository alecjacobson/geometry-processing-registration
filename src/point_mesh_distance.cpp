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
    N = Eigen::MatrixXd::Zero(X.rows(), X.cols());

    Eigen::MatrixXd N_all;
    igl::per_face_normals(VY, FY, Eigen::Vector3d(1, 1, 1).normalized(), N_all);

    for (int i = 0; i < X.rows(); i++) {

        double d = std::numeric_limits<double>::max();

        for (int j = 0; j < FY.rows(); j++) {
            const Eigen::RowVector3d x = X.row(i);

            const Eigen::Vector3d a = VY.row(FY(j, 0));
            const Eigen::Vector3d b = VY.row(FY(j, 1));
            const Eigen::Vector3d c = VY.row(FY(j, 2));

            double d_temp;
            Eigen::RowVector3d p_temp;
            point_triangle_distance(x, a, b, c, d_temp, p_temp);

            if (d_temp < d) {
                d = d_temp;
                P.row(i) = p_temp;
                N.row(i) = N_all.row(j);
            }
        }
    }

    D = (X - P).rowwise().norm();
}
