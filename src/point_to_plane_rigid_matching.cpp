#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"

#include <Eigen\Dense>
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
	// Replace with your code
	
	//Implements the least-square optimization found here
	//http://resources.mpi-inf.mpg.de/deformableShapeMatching/EG2011_Tutorial/slides/2.1%20Rigid%20ICP.pdf
	//Linarize the rotation in the same way, but write it out as a product of cross-product (cross-product matrix)
	//Then the matrix becomes simpler, i.e we minimize ((x-p)n + r(x.cross(n)) + tn)^2
	Eigen::VectorXd u;
	Eigen::MatrixXd A(X.rows(), 6);
	Eigen::VectorXd b(X.rows());

	Eigen::Vector3d p, n, x, res;	
	for (int i = 0; i < X.rows(); i++) {
		x = X.row(i);
		n = N.row(i);
		p = P.row(i);
		res = x.cross(n);
		A.row(i) << res(0), res(1), res(2), n(0), n(1), n(2);
		b.row(i) << -(x-p).dot(n);
	}

	u = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	Eigen::Matrix3d M;
	M << 1, -u(2), u(1),
		u(2), 1, -u(0),
		-u(1), u(0), 1;

	//Get the rotation and translation
	closest_rotation(M, R);	 
	t << u(3), u(4), u(5);
}
