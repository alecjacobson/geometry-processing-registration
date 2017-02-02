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
  // alpha, beta, gamma, tx, ty, tz
  Eigen::VectorXd u(6);
  
  // 6x6 Covariance matrix
  Eigen::MatrixXd C(6,6);
  C.setZero();
  
  // 6x1 b array
  Eigen::VectorXd b(6);
  b.setZero();
  
  Eigen::Vector3d c, x, n;
  Eigen::VectorXd cn(6);
  double xpn;
  for (int i = 0; i < X.rows(); i++) {
    x = X.row(i);
    n = N.row(i);
    c = x.cross(n);
    cn << c(0), c(1), c(2), N(i,0), N(i,1), N(i,2);
    C = C + (cn * cn.transpose());
    xpn = (X.row(i) - P.row(i)).dot(N.row(i));
    b = b + (xpn * cn);
  }
  
  // Solve for Cu = -b;
  u = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-1 * b);
  
  // Retrieve rotation matrix: alpha=u(0), beta=u(1), gamma=u(2)
  Eigen::MatrixXd M(3,3);
  M << 0, -u(2), u(1),
       u(2), 0, -u(0),
       -u(1), u(0), 0;
  closest_rotation(M, R);
  
  // Assign values of t
  t << u(3), u(4), u(5);
}
