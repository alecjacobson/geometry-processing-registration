#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d O;
    O << 1, 0, 0,
         0, 1, 0,
         0, 0, (U * V.transpose()).determinant();

    R = U * O * V.transpose();
}
