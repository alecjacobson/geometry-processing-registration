#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
	const Eigen::MatrixXd & X,
	const Eigen::MatrixXd & P,
	Eigen::Matrix3d & R,
	Eigen::RowVector3d & t)
{
	//I'm going to do the exact optimization, following 
	//https://content.sakai.rutgers.edu/access/content/group/7bee3f05-9013-4fc2-8743-3c5078742791/material/svd_ls_rotation.pdf

	Eigen::Vector3d x_centroid = X.colwise().mean();
	Eigen::Vector3d p_centroid = P.colwise().mean();
	
	Eigen::MatrixXd x_centered = X;
	Eigen::MatrixXd p_centered = P;

	x_centered.rowwise() -= x_centroid.transpose();
	p_centered.rowwise() -= p_centroid.transpose();

	Eigen::Matrix3d S = X.transpose()*P;
	closest_rotation(S, R);
	t = p_centroid - R*x_centroid;
}

