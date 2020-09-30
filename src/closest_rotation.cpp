#include "closest_rotation.h"

#include <Eigen/SVD>
#include <Eigen/LU>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd Vt = svd.matrixV().transpose();

    R << 1, 0, 0,
         0, 1, 0,
         0, 0, (U * Vt).determinant();
    R = (U * R) * Vt;
}
