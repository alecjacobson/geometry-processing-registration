#include "point_mesh_distance.h"
#include "point_triangle_distance.h"

#include <igl/per_face_normals.h>
#include <limits>
#include <vector>
#include <algorithm>


void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    D.resize(X.rows());
    P.resizeLike(X);
    N.resizeLike(X);

    auto ny = FY.rows();

    Eigen::MatrixXd NY;
    igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), NY);

    int min_idx;
    double d, min_d;
    Eigen::RowVector3d p, closest_p;

    for (int i = 0; i < X.rows(); ++i) {
        auto x = X.row(i);
        min_idx = 0;
        min_d = std::numeric_limits<int>::max();

        // Computes closest distance & points from `x` to every triangle `t`
        // Find triangle that minimizes point-to-mesh distance
        for (int j = 0; j < ny; ++j) {
            auto t = FY.row(j);
            point_triangle_distance(x, VY.row(t(0)), VY.row(t(1)), VY.row(t(2)), d, p);
            if (d < min_d) {
                min_d = d;
                min_idx = j;
                closest_p = p;
            }
        }
        
        // Saves closest distance, closest point, and normal
        D(i) = min_d;
        P.row(i) = closest_p;
        N.row(i) = NY.row(min_idx);
    }
}

