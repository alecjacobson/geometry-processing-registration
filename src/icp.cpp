#include "icp.h"

void icp(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const int num_iters,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t,
  Eigen::MatrixXd & X,
  Eigen::MatrixXd & P)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();
  X.resize(num_samples,3);
  P.resizeLike(X);
  for(int i = 0;i<X.rows();i++)
  {
    X.row(i) = VX.row(i%VX.rows());
    P.row(i) = VY.row(i%VY.rows());
  }
}
