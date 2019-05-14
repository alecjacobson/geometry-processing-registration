#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <iostream> 

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
      std::cout << "a" << std::endl;
  Eigen::MatrixXd A(3*X.rows(), 6);
  Eigen::MatrixXd L(X.rows(), 3*X.rows());
  Eigen::VectorXd C(3*X.rows());
  Eigen::MatrixXd M(3,3);
  Eigen::VectorXd U;
  Eigen::MatrixXd N0 = N.col(0).asDiagonal();
  Eigen::MatrixXd N1 = N.col(1).asDiagonal();
  Eigen::MatrixXd N2 = N.col(2).asDiagonal();
  Eigen::VectorXd T0 = Eigen::VectorXd::Zero(X.rows());
  Eigen::VectorXd T1 = Eigen::VectorXd::Ones(X.rows());

      std::cout << "b" << std::endl;
  L << N0, N1, N2;
        std::cout << "bb" << std::endl;
  C << X.col(0) - P.col(0),
     X.col(1) - P.col(1),
     X.col(2) - P.col(2);
           std::cout << "bc" << std::endl;
  A << T0, X.col(2), -X.col(1), T1, T0, T0,
     -X.col(2), T0, X.col(0), T0, T1, T0,
     X.col(1), -X.col(0), T0, T0, T0, T1;
    std::cout << "c" << std::endl;
  U = (A.transpose() * L.transpose() * L * A).inverse() * (-A.transpose() * L.transpose() * L * C);
    std::cout << "d" << std::endl;
  M << 0, -U(2), U(1),
     U(2), 0, -U(0),
     -U(1), U(0), 0;
    std::cout << "e" << std::endl;
  closest_rotation(M, R);
      std::cout << "f" << std::endl;
  t << U(3), U(4), U(5);

}
