#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h" 
#include <Eigen/Dense> // colPivHouseholderQr

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t) {
  
    // construct the matrices used in the minimization
    Eigen::MatrixXd A;
    Eigen::MatrixXd Nd;
    Eigen::MatrixXd Xp;
  
    // the first three columns of A gather the columns of X, with 0's along the diagonal
    A = Eigen::MatrixXd::Zero(3*X.rows(),6);
    A.block(0, 1, X.rows(), 1) = X.col(2);
    A.block(0, 2, X.rows(), 1) = -X.col(1);
    A.block(X.rows(), 0, X.rows(), 1) = -X.col(2);
    A.block(X.rows(), 2, X.rows(), 1) = X.col(0);
    A.block(2*X.rows(), 0, X.rows(), 1) = X.col(1);
    A.block(2*X.rows(), 1, X.rows(), 1) = -X.col(0);
    
    // the second three columns of A have X.rows() tall columns
    // of 1's
    A.block(0, 3, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());
    A.block(X.rows(), 4, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());
    A.block(2*X.rows(), 5, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());

    
    // Nd consists of concatenated diagonal matrices formed from the columns of N
    Nd.resize(N.rows(), 3*N.rows());
    Nd.block(0, 0, N.rows(), N.rows()) = N.col(0).asDiagonal(); 
    Nd.block(0, N.rows(), N.rows(), N.rows()) = N.col(1).asDiagonal(), 
    Nd.block(0, 2*N.rows(), N.rows(), N.rows()) = N.col(2).asDiagonal(); 
    
    // Xp consists of stacked columns of the form Xi - Pi (where Xi is the ith column
    // of X, etc.)
    Xp.resize(3*X.rows(), 1);
    Xp.block(0, 0, X.rows(), 1) = X.col(0) - P.col(0),
    Xp.block(X.rows(), 0, X.rows(), 1) = X.col(1) - P.col(1),
    Xp.block(2*X.rows(), 0, X.rows(), 1) = X.col(2) - P.col(2);
    
    // solve the energy minimizing system (au = b)
    Eigen::MatrixXd a = A.transpose()*Nd.transpose()*Nd*A;
    Eigen::VectorXd b = -A.transpose()*Nd.transpose()*Nd*Xp;
    Eigen::VectorXd u = a.colPivHouseholderQr().solve(b);
    
    // recover t from u and use the other parameters to construct the linearization matrix M.
    // Pass M to closest_rotation to compute the rotation matrix
    t << u(3), u(4), u(5);
    Eigen::Matrix3d M;
    M << 0, -u(2), u(1),
         u(2), 0, -u(0),
         -u(1), u(0), 0;
         
    M += Eigen::Matrix3d::Identity();             
    closest_rotation(M, R); 
}
