#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <random>

double getRandom() {
  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  return dist(gen);
}

void getCumulativeInfo(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, Eigen::VectorXd & C) {
  Eigen::VectorXd a(F.rows());
  igl::doublearea(V, F, a);
  double total_area = a.sum();
  a = a / total_area;
  igl::cumsum(a, a.rows(), C);
}

int getTriangle(const Eigen::VectorXd & C) {
  // Return the row number of the chosen triangle
  double g = getRandom();
  int count = 0;
  while (C(count) < g && count < C.rows()) {
    count++;
  }
  return count;
}


void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  
  // Construct cumulative array
  Eigen::VectorXd C(F.rows());
  getCumulativeInfo(V, F, C);
  
  int f_row, v1, v2, v3;
  double alpha, beta;
  for(int i = 0; i<X.rows(); i++) {
    
    // randomly pick a triangle from F
    f_row = getTriangle(C);
    v1 = F(f_row, 0);
    v2 = F(f_row, 1);
    v3 = F(f_row, 2);
    
    // randomly pick a point from the triangle
    alpha = getRandom();
    beta = getRandom();
    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }
    
    // set the values for x
    X.row(i) = V.row(v1) + alpha * (V.row(v2) - V.row(v1)) + beta * (V.row(v3) - V.row(v1));
  }
}

