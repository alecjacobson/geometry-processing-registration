#include "random_points_on_mesh.h"
#include <algorithm>
#include <stdlib.h>
#include <igl/doublearea.h>
#include <iostream>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n,3);
  
  int m = F.rows();
  
  Eigen::VectorXd A;
  igl::doublearea(V, F, A);
  A /= 2;
  A /= A.sum();
  
  // initialize the cumulative area array C
  double total_area;
  struct tr_area { int Fi; double area; };
  tr_area C[m];
  for (int i = 0; i < m; i++) {
    C[i] = tr_area{.Fi = i, .area = A[i] + (i > 0 ? C[i-1].area : 0)};
  }
  std::sort(C, C + m, 
    [](tr_area t1, tr_area t2){return t1.area < t2.area;});
  
  for (int i = 0; i < n; i++) {
  
    // obtain random numbers between 0 and 1
    double r = std::rand() / double(RAND_MAX);
    double alpha = std::rand() / double(RAND_MAX);
    double beta = std::rand() / double(RAND_MAX);
    
    // perform binary search to find the first entry C[t] in C > r
    int t = std::lower_bound(C, C + m, tr_area{.Fi = -1, .area = r}, 
      [](tr_area t1, tr_area t2){return t1.area < t2.area;}) - C;
    Eigen::Vector3d v1 = V.row(F(C[t].Fi, 0));
    Eigen::Vector3d v2 = V.row(F(C[t].Fi, 1));
    Eigen::Vector3d v3 = V.row(F(C[t].Fi, 2));
    
    // pick a random point on the triangle
    if (alpha + beta > 1) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }
    Eigen::Vector3d x = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
    
    X.row(i) = x;
  }
}

