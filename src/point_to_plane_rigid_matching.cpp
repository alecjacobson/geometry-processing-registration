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
    int r = X.rows();
    Eigen::MatrixXd A;
    A << Eigen::VectorXd::Zero(r), X.col(2), -X.col(1), Eigen::VectorXd::Ones(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r), -X.col(2), Eigen::VectorXd::Zero(r), X.col(0), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Ones(r), Eigen::VectorXd::Zero(r), X.col(1), -X.col(0), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Ones(r);
    Eigen::VectorXd B;
    B << X.col(0) - P.col(0), X.col(1) - P.col(1), X.col(2) - P.col(2);
    Eigen::MatrixXd Nd;
    Nd.block(0, 0, r, r) = N.col(0).asDiagonal();
    Nd.block(0, r, r, r) = N.col(1).asDiagonal();
    Nd.block(0, 2 * r, r, r) = N.col(2).asDiagonal();
    A = Nd * A;
    B = Nd * B;
    Eigen::VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * B);
    Eigen::Matrix3d M;
    M <<  1, u(2), -u(1), -u(2), 1, u(0), u(1), -u(0), 1;
    closest_rotation(M,R);
    t = Eigen::Vector3d(u(3), u(4), u(5));
}
