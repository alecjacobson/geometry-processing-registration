#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <iostream>
#include <random>

int find_index(Eigen::MatrixXd &C, double r) {
  // Handle edge case:
  if (r < C(0)) {
    return 0;
  }

  // Binary Search:
  int begin = 0;
  int end = C.rows() - 1;
  int mid = int((end + begin) / 2);

  while(begin + 1 < end) {
    double cur_mid = C(mid);
    if (cur_mid == r) {
      return mid + 1;
    }
    else if (cur_mid > r){
      if (C(mid-1)<r)
        return mid;
      end = mid;
    }
    else {
      begin = mid;
    }
    mid = int((end + begin) / 2);
  }
  return end;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n,3);

  // Compute Area:
  Eigen::MatrixXd all_area;
  Eigen::MatrixXd cum_area;
  igl::doublearea(V, F, all_area);
  igl::cumsum(all_area, 1, cum_area);
  cum_area.array().rowwise() /= cum_area.row(cum_area.rows()-1).array();

  // Uniform Random inspired by C++ library
  std::default_random_engine random_generator;
  std::uniform_real_distribution<double> uniform_distribution(0.0, 1.0);

  for(int i = 0; i < X.rows(); i ++) {
    // Find x:
    double threshold = uniform_distribution(random_generator);
    int ind = find_index(cum_area, threshold);
    Eigen::RowVectorXd v0 = V.row(F(ind, 0));
    Eigen::RowVectorXd v1 = V.row(F(ind, 1));
    Eigen::RowVectorXd v2 = V.row(F(ind, 2));

    double alpha = uniform_distribution(random_generator);
    double beta = uniform_distribution(random_generator);
    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }
    
    X.row(i) = v0 + alpha*(v1 - v0) + beta*(v2 - v0);
  }
}

