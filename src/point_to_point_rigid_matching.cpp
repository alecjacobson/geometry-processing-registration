#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  Eigen::Vector3d xCentroid, pCentroid;
  Eigen::MatrixXd xBar, pBar, M;

  xCentroid = X.colwise().mean();
  pCentroid = P.colwise().mean();
  xBar = X.rowwise() - xCentroid.transpose();
  pBar = P.rowwise() - pCentroid.transpose();

  M = (pBar.transpose() * xBar).transpose();
  closest_rotation(M, R);
  t = pCentroid - R * xCentroid;
}

