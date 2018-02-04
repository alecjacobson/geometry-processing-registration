#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    // Replace with your code
    Eigen::RowVector3d xmean = X.colwise().mean();
    Eigen::MatrixXd Xbar = X.rowwise() - xmean;
    
    Eigen::RowVector3d pmean = P.colwise().mean();
    Eigen::MatrixXd Pbar = P.rowwise() - pmean;
    Eigen::Matrix3d M = (Pbar.transpose() * Xbar).transpose();
    closest_rotation(M,R);
    
    t = pmean.transpose() - R*(xmean.transpose());
}

