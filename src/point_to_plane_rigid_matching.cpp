#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // alpha, beta, gamma, tx, ty, tz
  Eigen::VectorXd u(6);
  
  // kx6 matrix A
  int k = X.rows();
  Eigen::MatrixXd A(k,6);
  
  // k vector b
  Eigen::VectorXd b(k);
  
  // Construct A and b
  Eigen::Vector3d x, n, c, p;
  for (int i = 0; i < k; i++) {
    x = X.row(i);
    n = N.row(i);
    p = P.row(i);
    c = x.cross(n);
    A(i,0) = c(0);
    A(i,1) = c(1);
    A(i,2) = c(2);
    A(i,3) = n(0);
    A(i,4) = n(1);
    A(i,5) = n(2);
    b(i) = (x-p).dot(n);
  }

  // Solve for Ax = b;
  u = (A).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-1* b);
  
  // Retrieve rotation matrix: alpha=u(0), beta=u(1), gamma=u(2)
  Eigen::MatrixXd M(3,3);
  M << 0, -u(2), u(1),
       u(2), 0, -u(0),
       -u(1), u(0), 0;
  closest_rotation(M, R);
  
  // Assign values of t
  t << u(3), u(4), u(5);
  std::cout << t << std::endl;
}
