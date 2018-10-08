#include <Eigen/Sparse>
#include <Eigen/Cholesky>

#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code

  typedef Eigen::Triplet<double> T;
  int k = X.rows();
  
  // Compute A and Nbar = [diag(N1) diag(N2) diag(N3)]
  std::vector<T> tripletListA;
  tripletListA.reserve(9*k);
  std::vector<T> tripletListNbar;
  tripletListNbar.reserve(3*k);
  for (int i = 0; i < k; ++i) {
    // -X3
    tripletListA.push_back(T(k + i, 0, -X(i,2)));

    // X2
    tripletListA.push_back(T(2*k + i, 0, X(i,1)));    

    // X3
    tripletListA.push_back(T(i, 1, X(i,2)));

    // -X1
    tripletListA.push_back(T(2*k + i, 1, -X(i,0)));

    // -X2
    tripletListA.push_back(T(i, 2, -X(i,1)));

    // X1
    tripletListA.push_back(T(k + i, 2, X(i,0)));

    // 1's
    tripletListA.push_back(T(i, 3, 1));
    tripletListA.push_back(T(k + i, 4, 1));
    tripletListA.push_back(T(2*k + i, 5, 1));

    // Nbar
    tripletListNbar.push_back(T(i, i, N(i,0)));
    tripletListNbar.push_back(T(i, k + i, N(i,1)));
    tripletListNbar.push_back(T(i, 2*k + i, N(i,2)));
  }

  Eigen::SparseMatrix<double> A(3*k, 6);
  A.setFromTriplets(tripletListA.begin(), tripletListA.end());
  Eigen::SparseMatrix<double> Nbar(k, 3*k);
  Nbar.setFromTriplets(tripletListNbar.begin(), tripletListNbar.end());

  // Form vector b = [X1 - P1; X2 - P2; X3 - P3]
  Eigen::MatrixXd temp = X - P;
  Eigen::VectorXd b(Eigen::Map<Eigen::VectorXd>(temp.data(), temp.cols()*temp.rows()));

  // Solve linear system A'*Nbar'*NbarA*u = -A'*Nbar'*Nbar*b for u
  // Abar*u = bbar
  Eigen::SparseMatrix<double> temp1 = A.transpose()*Nbar.transpose()*Nbar;
  Eigen::SparseMatrix<double> temp2 = temp1*A;
  Eigen::MatrixXd Abar = temp2;
  Eigen::VectorXd u = Abar.llt().solve(-temp1*b);

  // Compute M and project to SO(3) to get R
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
  M(1,0) += u(2);
  M(0,1) -= u(2);
  M(0,2) += u(1);
  M(2,0) -= u(1);
  M(2,1) += u(0);
  M(1,2) -= u(0);
  closest_rotation(M, R);

  // Grab t from u
  t = u.tail<3>().transpose();
}
