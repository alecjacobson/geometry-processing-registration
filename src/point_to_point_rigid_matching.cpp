#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
	// Closed-form point-to-point minimizer
	Eigen::RowVector3d X_centroid, P_centroid;
	Eigen::MatrixXd X_bar, P_bar;
	
	// Compute centroids
	X_centroid = X.colwise().mean();
	P_centroid = P.colwise().mean();
	
	// Construct P_bar and X_bar
	X_bar = X.rowwise() - X_centroid;
	P_bar = P.rowwise() - P_centroid;
	Eigen::MatrixXd M = X_bar.transpose()*P_bar;

	// Find optimal R
	closest_rotation(M, R);

	// Find optimal t
	t = P_centroid.transpose() - R*(X_centroid.transpose());
}

