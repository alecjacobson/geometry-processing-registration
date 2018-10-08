#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>

#include <random>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
    Eigen::MatrixXd areas(F.rows(), 1);

    igl::doublearea(V, F, areas);

    Eigen::VectorXd C(F.rows());

    igl::cumsum(areas, 1, C);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    X.resize(n, 3);
    for (int i = 0; i < X.rows(); i++) {

        double gamma = distribution(generator);

        int T=0;
        while (C(T)/C(C.rows()-1) < gamma)
            T++;

        Eigen::Vector3d v1 = V.row(F(T, 0));
        Eigen::Vector3d v2 = V.row(F(T, 1));
        Eigen::Vector3d v3 = V.row(F(T, 2));

        double alpha = distribution(generator);
        double beta = distribution(generator);

        if ((alpha + beta) > 1) {
            alpha = 1 - alpha;
            beta = 1 - beta;
        }

        Eigen::Vector3d x = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
        X.row(i) = x;
    }
}

