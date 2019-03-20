#include "hausdorff_lower_bound.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  // Replace with your code
    Eigen::MatrixXd X;
    random_points_on_mesh(n,VX,FX,X);
    Eigen::VectorXd D;
    Eigen::MatrixXd P,N;
    point_mesh_distance(X,VY,FY,D,P,N);
    return D.maxCoeff();
}
