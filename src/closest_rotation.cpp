#include "closest_rotation.h"
#include <Eigen/Dense>
#include <Eigen/LU>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = (svd.matrixV()).transpose();
    Eigen::Matrix3d O = Eigen::Matrix3d::Identity();
    O(2,2) = (U * Vt).determinant();
    R = U * O * Vt;
}
