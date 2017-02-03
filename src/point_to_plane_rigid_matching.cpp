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
  // u: alpha, beta, gamma, t1, t2, t3
  Eigen::VectorXd u(6);
  
  Eigen::MatrixXd C(X.rows(),6);
  Eigen::VectorXd b(X.rows());
  C.setZero();
  b.setZero();
  
  Eigen::Vector3d x, p, n, c;
  Eigen::VectorXd cov(6);
  for (int i = 0; i < X.rows(); i++) {
    x = X.row(i);
    n = N.row(i);
    p = P.row(i);
    c = x.cross(n);
    C(i,0) = c(0);
    C(i,1) = c(1);
    C(i,2) = c(2);
    C(i,3) = n(0);
    C(i,4) = n(1);
    C(i,5) = n(2);
    b(i) = n.dot(p-x);
  }
  
  // Solve for Cu = b
  u = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  
  // Construct M, R
  Eigen::Matrix3d M;
  M << 0, -u(2), u(1),
       u(2), 0, -u(0),
       -u(1), u(0), 0;
  closest_rotation(M,R);
  
  // Construct t
  t << u(3), u(4), u(5);
}
