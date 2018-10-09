#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <iostream>
#include <random>

int find_random_index(Eigen::MatrixXd & C, double r, int start, int end) {
// identifying the first entry in C whose value is greater than a uniform random variable γ.
//int start = 0;
//int end = C.rows() - 1;
while (start != end) {
  int mid = (start + end) / 2;
  double c = C(mid,0);
  if (c >= r) {
    //start = start;
    end = mid;
    //mid = (start + end) / 2;
  }
  else if (c < r)
  {
    start = mid;
    //end = end;
  //mid = (start + end) / 2;
  }
}

if (C(0,0) > r) {
  return 0;
} 
else {
  return end;
}


// int mid = (start + end) / 2;
// double c = C(mid,0);
// if (c > r) {
//   start = start;
//   end = mid;
//   //mid = (start + end) / 2;
// }

// else if (c < r)
// {
//   start = mid;
//   end = end;
//   //mid = (start + end) / 2;
// }

// else {
//   start = end;
// }

// if (start < end) {
//   find_random_index(C,r,start,end);
// }
// return mid;
}


void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

  // Area-weighted random sampling of triangles
  Eigen::MatrixXd A,C;
  igl::doublearea(V,F,A);
  igl::cumsum(A,1,C);
  C = 1.0 * C / C(C.rows()-1,0);

  // Uniform random sampling of a single triangle
  // a b
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  double a;
  double b;
  double r;
  int index;
 
  // random point in a single triangle mesh
  Eigen::RowVector3d x;
  Eigen::RowVector3i single;
  Eigen::RowVector3d v1,v2,v3;

  for (int i=0; i < X.rows(); i++){
    r = distribution(generator);
    index = find_random_index(C,r,0,C.rows()-1);
    single = F.row(index);
    v1 = V.row(single[0]);
    v2 = V.row(single[1]);
    v3 = V.row(single[2]);
    a = distribution(generator);
    b = distribution(generator);
    if (a + b > 1) {
      a = 1 - a;
      b = 1 - b;
    }
    x = v1 + a * (v2 - v1) + b * (v3 - v1);
    X.row(i) = x;
  }

}

