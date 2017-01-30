#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());
}

