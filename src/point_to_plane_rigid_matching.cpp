#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // u: alpha, beta, gamma, t1, t2, t3
  Eigen::VectorXd u(6);
  
  Eigen::MatrixXd C(6,6);
  Eigen::VectorXd b(6);
  C.setZero();
  b.setZero();
  
  Eigen::Vector3d x, p, n, c;
  Eigen::VectorXd cov(6);
  for (int i = 0; i < X.rows(); i++) {
    x = X.row(i);
    n = N.row(i);
    p = P.row(i);
    c = x.cross(n);
    cov << c(0), c(1), c(2), n(0), n(1), n(2);
    C += cov * cov.transpose();
    b += (p-x).dot(n) * cov;
  }
  
  // Solve for Cu = b
  u = C.ldlt().solve(b);
  
  // Construct M, R
  Eigen::Matrix3d M;
  M << 1, -u(2), u(1),
       u(2), 1, -u(0),
       -u(1), u(0), 1;
  closest_rotation(M,R);
  
  // Construct t
  t << u(3), u(4), u(5);
}
