#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // // Replace with your code
  // R = Eigen::Matrix3d::Identity();

	// Compute the SVD of the matrix M
    Eigen::MatrixXd M_Xd = M;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Compute the product of the left and right orthogonal matrices
	Eigen::MatrixXd UVT = svd.matrixU()*svd.matrixV().transpose();
	double detUVT = UVT.determinant();

	Eigen::Matrix3d Omega;
	Omega.resize(3,3);

	// Populate the Omega matrix, as per notes
	for (int ii = 0; ii < 3; ii++)
	{
		for (int jj = 0; jj < 3; jj++)
		{
			Omega(ii,jj) = 0.0;

			if (ii == jj && ii != 2)
				Omega(ii,jj) = 1.0;
			if (ii == 2 && jj == 2)
				Omega(ii,jj) = detUVT;
		}
	}

	// Finally, compute the rotation matrix
	R = svd.matrixU()*Omega*svd.matrixV().transpose();
	// std::cout << R << std::endl;

	return;

}
