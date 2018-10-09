#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>

int random_triangle(Eigen::VectorXd cumsum) 
{
    double gamma = (double)rand() / RAND_MAX;
    int L = 0;
    int R = cumsum.size() - 1;
    while (L != R) 
    {
        int m = (L + R) / 2;
        if (cumsum(m) <= gamma) 
        {
            L = m + 1;
        }
        else 
        {
            R = m;
        }
    }

    return L;
}

Eigen::Vector3d random_point_in_triangle(
    Eigen::Vector3d v1,
    Eigen::Vector3d v2,
    Eigen::Vector3d v3) 
{
    double alpha = (double)rand() / RAND_MAX;
    double beta = (double)rand() / RAND_MAX;

    if (alpha + beta > 1) 
    {
        alpha = 1 - alpha;
        beta = 1 - beta;
    }

    return v1 + alpha * (v2 - v1) + beta * (v3 - v1);
}

void random_points_on_mesh(
    const int n,
    const Eigen::MatrixXd & V,
    const Eigen::MatrixXi & F,
    Eigen::MatrixXd & X)
{
    Eigen::VectorXd dblA, cumsum;
    igl::doublearea(V, F, dblA);
    igl::cumsum(dblA, 1, cumsum);
    double totalA = dblA.sum();
    cumsum = cumsum / totalA;

    X.resize(n, 3);

    for (int i = 0; i < n; i++) 
    {
        Eigen::Vector3i t = F.row(random_triangle(cumsum));
        X.row(i) = random_point_in_triangle(V.row(t[0]), V.row(t[1]), V.row(t[2]));
    }
}
