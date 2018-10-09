#include "closest_rotation.h"

void closest_rotation(
    const Eigen::Matrix3d & M,
    Eigen::Matrix3d & R)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV().transpose();
    Eigen::MatrixXd UV = U * V;
    Eigen::Vector3d omega(1, 1, UV.determinant());
    Eigen::Matrix3d Omega = omega.asDiagonal();
    R = U * Omega * V;
}
