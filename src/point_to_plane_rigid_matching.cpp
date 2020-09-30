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
    // Construct matrix A
    int k = X.rows();
    Eigen::VectorXd u(6);
    Eigen::MatrixXd A(3*k, 6);
    Eigen::MatrixXd b(3*k, 1);
    Eigen::MatrixXd dN(k, 3*k);

    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(k);
    Eigen::VectorXd ones  = Eigen::VectorXd::Ones(k);

    A << zeros, X.col(2), -X.col(1), ones, zeros, zeros,
         -X.col(2), zeros, X.col(0), zeros, ones, zeros,
         X.col(1), -X.col(0), zeros, zeros, zeros, ones;

    b << X.col(0) - P.col(0),
         X.col(1) - P.col(1),
         X.col(2) - P.col(2);

    dN << Eigen::MatrixXd(N.col(0).asDiagonal()), 
          Eigen::MatrixXd(N.col(1).asDiagonal()),
          Eigen::MatrixXd(N.col(2).asDiagonal());

    A = dN * A;
    b = dN * b;

    // solve for u
    u = (A.transpose()*A).inverse() * (-A.transpose()*b);

    // fill R and t
    Eigen::Matrix3d M;
    M << 1, -u(2), u(1),
         u(2), 1, -u(0),
         -u(1), u(0), 1;
    closest_rotation(M, R);
    t = u.tail(3);
}
