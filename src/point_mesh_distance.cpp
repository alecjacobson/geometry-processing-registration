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

    auto nx = X.rows();
    auto ny = FY.rows();

    Eigen::MatrixXd NY;
    igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), NY);

    int min_idx;
    double d;
    Eigen::RowVector3d p;
    Eigen::VectorXd closest_distances(ny);
    Eigen::MatrixXd closest_points(ny, 3);

    for (int i = 0; i < nx; ++i) {
        auto x = X.row(i);

        // Computes closest distance & points from `x` to every triangle `t`
        for (int j = 0; j < ny; ++j) {
            auto t = FY.row(j);
            point_triangle_distance(x, VY.row(t(0)), VY.row(t(1)), VY.row(t(2)), d, p);
            closest_distances(j) = d;
            closest_points.row(j) = p;
        }
        
        // Find triangle that minimizes point-to-mesh distance
        d = std::numeric_limits<int>::max();
        min_idx = 0;
        for (int j = 0; j < ny; ++j) {
            if (closest_distances(j) < d) {
                d = closest_distances(j);
                min_idx = j;
            }
        }
        
        // Saves closest distance, closest point, and normal
        D(i) = d;
        P.row(i) = closest_points.row(min_idx);
        N.row(i) = NY.row(min_idx);
    }
}

