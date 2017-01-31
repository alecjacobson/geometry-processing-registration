#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <iostream>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();


  Eigen::RowVector3d cX = X.colwise().sum() / X.rows();
  Eigen::RowVector3d cP = P.colwise().sum() / X.rows();
  Eigen::MatrixXd bX = X.rowwise() - cX;
  Eigen::MatrixXd bP = P.rowwise() - cP;
  Eigen::Matrix3d M = bP.transpose() * bX;

  closest_rotation(M,R);



  t = (cP -  cX * R.transpose());

  //TODO: Why is this the best (R here R.T for t?
  R = R.transpose().eval();
  std::cout << t.transpose() << std::endl;

}

