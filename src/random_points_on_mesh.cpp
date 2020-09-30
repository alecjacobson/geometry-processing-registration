#include "random_points_on_mesh.h"

#include <random>
#include <vector>
#include <algorithm>
#include <igl/doublearea.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
    X.resize(n, 3);

    // Area-weighted random sampling of triangles

    Eigen::MatrixXd A;      // (F.rows(), 1)
    igl::doublearea(V, F, A);

    auto total_area = A.sum();
    auto C = std::vector<double>(A.rows());

    for (int i = 0; i < C.size(); ++i) {
        if (i == 0) {
            C[i] = A(i);
        } else {
            C[i] = C[i-1] + A(i);
        }
    }
    for (int i = 0; i < C.size(); ++i) {
        C[i] = C[i] / total_area;
    }

    // Uniform random sampling of n triangles

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> unif(0., 1.);

    int v, T;
    double alpha, beta;
    std::vector<double>::iterator upper;
    Eigen::RowVector3i triangle;
    Eigen::RowVector3d v1, v2, v3;

    for (int i = 0; i < n; ++i) {
        upper = std::upper_bound(C.begin(), C.end(), unif(gen));
        T = upper - C.begin();
        triangle = F.row(T);

        alpha = unif(gen);
        beta  = unif(gen);

        if (alpha + beta > 1) {
            alpha = 1. - alpha;
            beta  = 1. - beta;
        }

        v = rand() % 3;
        v1 = V.row(triangle(v));
        v2 = V.row(triangle((v+1) % 3));
        v3 = V.row(triangle((v+2) % 3));

        X.row(i) = v1.array() + \
            alpha*(v2.array() - v1.array()) + \
             beta*(v3.array() - v1.array());
    }
}

