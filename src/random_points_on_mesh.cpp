#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  // Get the area matrix
  Eigen::MatrixXd A(n,1);
  igl::doublearea(V, F, A);
  // Get the cumulative sum
  Eigen::MatrixXd C(n,1);
  igl::cumsum(A,1,C);

  for(int i=0; i < n; i++){
    int triangle_idx = get_random_triangle(C, n);
    Eigen::RowVector3d triangle_pt = get_random_point(triangle_idx, V, F);
    X.row(i) = triangle_pt;
  }
}

int get_random_triangle(
  const Eigen::MatrixXd & C,
  const int n)
{
  int gamma = rand();
  int c_idx = n/2;
  while(C(c_idx) < gamma){
    c_idx *= 2;
  }
  return c_idx;
}

Eigen::RowVector3d get_random_point(
  const int triangle_idx,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F)
{
  Eigen::RowVector3d v1 = V.row(F(triangle_idx,0));
  Eigen::RowVector3d v2 = V.row(F(triangle_idx,1));
  Eigen::RowVector3d v3 = V.row(F(triangle_idx,2));

  float a = rand(); float b = rand();
  if(a + b > 1){
    a = 1 - a;
    b = 1 - b;
  }
  return v1 + a*(v2-v1) + b*(v3-v1);
}
