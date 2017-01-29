#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & B,
  Eigen::VectorXi & FI)
{
  // REPLACE WITH YOUR CODE:
  B.resize(n,3);
  B.col(0).setConstant(1);
  B.col(1).setConstant(0);
  B.col(2).setConstant(0);
  FI = Eigen::VectorXi::LinSpaced(n,0,n-1);
}

