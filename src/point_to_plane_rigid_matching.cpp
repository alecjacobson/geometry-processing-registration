#include "point_to_plane_rigid_matching.h"
#include <Eigen/LU>
#include <Eigen/Dense>
#include "closest_rotation.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();


  Eigen::MatrixXd N_x_diag(N.rows(), N.rows());
  Eigen::MatrixXd N_y_diag(N.rows(), N.rows());
  Eigen::MatrixXd N_z_diag(N.rows(), N.rows());
  N_x_diag = Eigen::MatrixXd(N.col(0).asDiagonal());
  N_y_diag = Eigen::MatrixXd(N.col(1).asDiagonal());
  N_z_diag = Eigen::MatrixXd(N.col(2).asDiagonal());


  Eigen::MatrixXd N_diag_concat(N.rows(), 3*N.rows());
  N_diag_concat << N_x_diag, N_y_diag, N_z_diag;


  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*N.rows(), 6);

  Eigen::VectorXd X_minus_P(3*N.rows());

  // iterate over all x
  for (int i=0; i<X.rows(); i++) {

    X_minus_P(i) = X(i, 0) - P(i, 0);
    X_minus_P(X.rows()+i) = X(i, 1) - P(i, 1);
    X_minus_P(2*X.rows() + i) = X(i, 2) - P(i, 2);

    A(X.rows() + i, 0) = -1.0 * X(i, 2);
    A(2*X.rows() + i, 0) = X(i, 1);
    A(i, 1) = X(i, 2);
    A(2*X.rows() + i, 1) = -1.0 * X(i, 0);
    A(i, 2) = -1.0 * X(i, 1);
    A(X.rows() + i, 2) = X(i, 0);

    A(i, 3) = 1;
    A(X.rows() + i, 4) = 1;
    A(2*X.rows() + i, 5) = 1;
  }

  Eigen::MatrixXd A_new =  N_diag_concat * A;
  Eigen::VectorXd X_minus_P_new = N_diag_concat * X_minus_P;

  Eigen::VectorXd U(6);
  
  
  U = (A_new.transpose()*A_new).colPivHouseholderQr().solve(-1.0*A_new.transpose()*X_minus_P_new);


  // assign answer for t
  t(0) = U(3); t(1) = U(4); t(2) = U(5);

  // Create a matrix M
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
  M(0, 1) = -1.0 * U(2);
  M(0, 2) = U(1);
  M(1, 0) = U(2);
  M(1, 2) = -1.0 * U(0);
  M(2, 0) = -1.0 * U(1);
  M(2, 1) = U(0);

  // now find the closest rotation
  closest_rotation(
  M,
  R);

}
