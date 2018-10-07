#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/LU>

using namespace Eigen;

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    MatrixXd Vt = V.transpose();
    double det = (U * Vt).determinant();
    Eigen::Matrix3d Omega;
    Omega << 1, 0, 0,
             0, 1, 0,
             0, 0, det;
    R = U * Omega * Vt;
}

