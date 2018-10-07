#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>
#include <iostream>

using namespace Eigen;
using namespace std;

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n,3);
  VectorXd doubleArea, cumArea;
  igl::doublearea(V, F, doubleArea);
  igl::cumsum(doubleArea / 2.0, 1, cumArea);
  double totalArea = cumArea(F.rows() - 1);
  cumArea = cumArea * 1.0 / totalArea;

  double alpha, beta;
  default_random_engine generator;
  uniform_real_distribution<double> distribution(0.0, 1.0);

  int first, last, mid;
  int tri_index;

  RowVector3d v1, v2, v3;

  for (int i = 0; i < X.rows(); i++) {
    // randomly pick a triangle
    double tri_sample = distribution(generator);
    first = 0;
    last = F.rows() - 1;
    // binary search
    while (last - first > 1) {
      mid = (first + last) / 2;
      if (tri_sample <= cumArea(mid)) {
        last = mid;
      }
      else {
        first = mid;
      }
    }
    if (tri_sample <= cumArea(0)) {
      tri_index = 0;
    }
    else {
      tri_index = last;
    }

    alpha = distribution(generator);
    beta = distribution(generator);

    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }

    v1 = V.row(F(tri_index, 0));
    v2 = V.row(F(tri_index, 1));
    v3 = V.row(F(tri_index, 2));

    X.row(i) = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
  }

}

