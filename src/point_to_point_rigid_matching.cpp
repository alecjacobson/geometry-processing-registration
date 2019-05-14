#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <igl/polar_svd.h>
#include <iostream> 

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
	Eigen::Matrix3d M;
	Eigen::RowVector3d P_hat;
	Eigen::RowVector3d X_hat;
	Eigen::MatrixXd P_norm;
	Eigen::MatrixXd X_norm;
 	
    std::cout << "t1" << std::endl;
 	X_hat = X.colwise().sum();
 	P_hat = P.colwise().sum();
 	X_hat /= X.rows();
 	P_hat /= P.rows();
 	X_norm = X.rowwise() - X_hat;
 	P_norm = P.rowwise() - P_hat;
 	std::cout << P_norm.rows() << std::endl;
 	std::cout << "t2" << std::endl;
    std::cout << X_norm.rows() << std::endl;
 	M = P_norm.transpose() * X_norm;
 	std::cout << M << std::endl;
 	closest_rotation(M, R);
 	    std::cout << "t4" << std::endl;
    t = P_hat.transpose() - (R * X_hat.transpose());
        std::cout << "t5" << std::endl;


}

