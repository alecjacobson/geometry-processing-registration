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
  int k = X.rows();
  Eigen::VectorXd X1, X2, X3, kOnes, kZeroes, u;
  Eigen::MatrixXd N1, N2, N3;
  Eigen::MatrixXd diagN(k, 3 * k);
  Eigen::VectorXd diff(3 * k);
  Eigen::MatrixXd A(3 * k, 6);

  // construct sub-sections of matrices
  N1 = N.col(0).asDiagonal();
  N2 = N.col(1).asDiagonal();
  N3 = N.col(2).asDiagonal();

  X1 = X.col(0);
  X2 = X.col(1);
  X3 = X.col(2);

  kOnes = Eigen::VectorXd::Ones(k);
  kZeroes = Eigen::VectorXd::Zero(k);

  // construct matrices
  diagN << N1, N2, N3;
  
  A << kZeroes, X3, -X2, kOnes, kZeroes, kZeroes,
       -X3, kZeroes, X1, kZeroes, kOnes, kZeroes,
       X2, -X1, kZeroes, kZeroes, kZeroes, kOnes;
  
  diff << X1 - P.col(0),
          X2 - P.col(1),
          X3 - P.col(2);
  
  // solve for u!
  A = diagN * A;
  diff = diagN * diff;
  u = (A.transpose() * A).inverse() * (-A.transpose() * diff);

  // construct M
  Eigen::MatrixXd M(3, 3);
  M << 1.0, u(2), -u(1),
       -u(2), 1.0, u(0),
       u(1), -u(0), 1.0;

  // get R and t!
  closest_rotation(M, R);
  t = u.tail<3>();
}
