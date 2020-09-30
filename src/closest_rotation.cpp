#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d omega_star;
  double dt = (U*V.transpose()).determinant();

  omega_star << 1, 0, 0,
                0, 1, 0,
                0, 0, dt;

  R = U*omega_star*V.transpose();
}