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
	
	// Implements the closed form point-point matching solution:
	// Heavily borrowed from this resource: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	// with our point weights being set uniformly across all samples.

	//1. Find the centroid https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
	Eigen::RowVector3d x_bar = X.colwise().mean();
	Eigen::RowVector3d p_bar = P.colwise().mean();

	//2. Compute the centered vectors for each mesh
	Eigen::MatrixXd cent_X(X), cent_P(P);
	cent_X.rowwise() -= x_bar;
	cent_P.rowwise() -= p_bar;

	//3. Compute 3 x 3 covariance matrix. Note the reference has their matrices in 3 x k
	// ours is k x 3. So their UV^T = (P^T)(X^T)^T = P^T * X. Also note, they have
	// flipped the points they are aligning.
	Eigen::Matrix3d M = (cent_P.transpose()) * cent_X;

	//4. Get closest rotation and translation
	closest_rotation(M, R);
	t = p_bar - x_bar * R;

}

