#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <random>
#include <vector>
#include "point_triangle_distance.h"
#include <iostream>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n, 3);

  Eigen::VectorXd triangleArea(F.rows());
  igl::doublearea(V, F, triangleArea);

  Eigen::VectorXd csArea(F.rows());
  igl::cumsum(triangleArea / 2.0, 1, csArea);

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  std::vector<double> vec(csArea.data(), csArea.data() + csArea.size());

  for (int i = 0; i < n; i++) {
    // Sample triangle
    double gamma = distribution(generator);
    int loc = (int) (std::lower_bound(vec.begin(), vec.end(), gamma) - vec.begin());

    // Sample points
    double alpha = distribution(generator);
    double beta = distribution(generator);
    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }

    Eigen::Vector3d v1 = V.row(F(loc, 0));
    Eigen::Vector3d v2 = V.row(F(loc, 1));
    Eigen::Vector3d v3 = V.row(F(loc, 2));

    Eigen::Vector3d x = v1 + alpha * (v2 - v1) + beta * (v3 - v1);

    X.row(i) = x;

//    double d;
//    Eigen::RowVector3d p;
//    point_triangle_distance(x, v1, v2, v3, d, p);
//    std::cout << d<<" "<<p<<std::endl;
  }
}

