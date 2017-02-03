#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>

using namespace Eigen;

void makeRelative(MatrixXd &M, RowVectorXd mean)
{
	assert(M.cols() == mean.size());

	int n = M.rows(), m = M.cols();
	for (int i = 0; i < m; ++i)
		M.col(i) -= mean(i)*VectorXd::Ones(n);
}

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
	RowVector3d x_mean(X.col(0).mean(), X.col(1).mean(), X.col(2).mean()),
		p_mean(P.col(0).mean(), P.col(1).mean(), P.col(2).mean());

	MatrixXd X_bar(X), P_bar(P);

	//move both centroids to origin
	makeRelative(X_bar, x_mean);
	makeRelative(P_bar, p_mean);

	Matrix3d M = P_bar.transpose()*X_bar;
	closest_rotation(M, R);
	t = p_mean - x_mean*R.transpose();
}

