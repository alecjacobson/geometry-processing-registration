#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>
#include <iostream>

int binary_search_upper_bound(
  const Eigen::VectorXd & v,
  int lower,
  int upper,
  const double val)
{
  int n = upper - lower;
  int ind = upper;
  if (n >= 1) {
    ind = lower + (upper - lower)/2;
    if (v(ind) <= val) {
      return binary_search_upper_bound(v, ind + 1, upper, val);
    }
    if (v(ind - 1) <= val) {
      return ind;
    }
    return binary_search_upper_bound(v, lower, ind - 1, val);
  }
  return ind;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{

  // create vector of triangle areas
  Eigen::MatrixXd areas(F.rows(), 1);
  igl::doublearea(V, F, areas);
  // cumsum all triangle areas
  Eigen::MatrixXd cumulativeAreas(F.rows(), 1);
  igl::cumsum(areas, 1, cumulativeAreas);
  // divide by last entry (which should be total area)
  Eigen::MatrixXd cumulativeRelativeAreas(F.rows(), 1);
  cumulativeRelativeAreas = cumulativeAreas / cumulativeAreas.row(cumulativeAreas.rows() - 1).value();

  // set up variables...
  X.resize(n,3);
  Eigen::MatrixXd v0, v1, v2;
  double alpha, beta, gamma;
  int ind;
  std::random_device seed;
  std::mt19937 gen(seed());
  std::uniform_real_distribution<> dist(0.0, 1.0);

  for(int i = 0;i<X.rows();i++) {
    // 1) pick random triangle
    gamma = dist(gen);

    // find index of first entry in C whose value is greater than a uniform random variable γ
    // ... binary search
    ind = binary_search_upper_bound(cumulativeRelativeAreas.col(0), 0, cumulativeRelativeAreas.rows() - 1, gamma);
    if (ind == -1) ind = cumulativeRelativeAreas.rows() - 1;
    

    // get vertices of the randomly selected triangle
    v0 = V.row(F(ind, 0));
    v1 = V.row(F(ind, 1));
    v2 = V.row(F(ind, 2));

    // 2) randomly sample a point in the triangle
    // uniform random sample
    alpha = dist(gen);
    beta = dist(gen);
    // reflection 
    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }
    // x = v0 + alpha(v1 - v0) + beta(v2 - v0)
    X.row(i) = v0 + (alpha * (v1 - v0)) + (beta * (v2 - v0)); 
  }
}

