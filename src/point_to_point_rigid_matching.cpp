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
  
	Eigen::RowVector3d xHat = X.colwise().sum() / X.rows();
	Eigen::RowVector3d pHat = P.colwise().sum() / P.rows();

	Eigen::MatrixXd XBar = X - xHat.replicate(X.rows(), 1);
	Eigen::MatrixXd PBar = P - xHat.replicate(P.rows(), 1);

	Eigen::Matrix3d M = XBar.transpose()*PBar;

	closest_rotation(M,R);

	t = pHat.transpose() - R*xHat.transpose();
}

