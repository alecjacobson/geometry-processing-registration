#include "random_points_on_mesh.h"
#include <iostream>
#include <random>
#include "igl/doublearea.h"
#include "igl/cumsum.h"

int return_index(Eigen::VectorXd C, double r)
{
  for(int i = 0; i < C.rows(); i++)
  {
    if (C(i) / C(C.rows() - 1) >= r)
        return i;
  }
}

void random_points_on_mesh(
    const int n,
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    Eigen::MatrixXd &X)
{
  X.resize(n, 3);

  Eigen::MatrixXd list;
  Eigen::VectorXd C;
  igl::doublearea(V, F, list);
  igl::cumsum(list, 1, C);

  //uniformed distribution from http://www.cplusplus.com/reference/random/uniform_real_distribution/
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  for (int i = 0; i < X.rows(); i++)
  {
    double a = distribution(generator);
    double b = distribution(generator);
    double r = distribution(generator);

    if (a + b > 1.0)
    {
      a = 1 - a;
      b = 1 - b;
    }
    int idx = return_index(C, r);
    X.row(i) = V.row(F(idx, 0)) + a * (V.row(F(idx, 1)) - V.row(F(idx, 0))) + b * (V.row(F(idx, 2)) - V.row(F(idx, 0)));
  }
}
