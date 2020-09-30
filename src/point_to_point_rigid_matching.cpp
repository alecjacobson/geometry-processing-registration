#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    Eigen::MatrixXd xbar, pbar;
    xbar = X.rowwise() - X.colwise().mean();
    pbar = P.rowwise() - P.colwise().mean();
    Eigen::Matrix3d M = (pbar.transpose() * xbar).transpose();
    closest_rotation(M, R);
    t = (P.colwise().mean()).transpose() - R*((X.colwise().mean()).transpose());
}
