#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // solving using the linearization:
  //
  // min ∑‖ Rx + t - Py(T(x)) ‖²
  //
  //      ‖      / X₁ - P₁ \ ‖²
  // min ∑‖ Au + | X₂ - P₂ | ‖    ,   uϵℝ⁶ => (α,β,ɣ, tϵℝ³)
  //      ‖      \ X₃ = P₃ / ‖ 
  //
  //             [X_i - P_i]
  //
  //           /  0    X₃  -X₂  1  0  0 \
  // where A = | -X₃   0    X₁  0  1  0 |
  //           \  X₂  -X₁   0   0  0  1 /
  //
  // using the linearization, minimum occurs at
  // (AᵀA)u = (-Aᵀ [X_i - P_i] )
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*X.rows(), 6);

  // add the X_i column vectors to the A:
  // starting at (0,1) and of size (X.rows(), 1)
  A.block(0, 1, X.rows(), 1) =  X.col(2);
  A.block(0, 2, X.rows(), 1) = -X.col(1);
  
  A.block(X.rows(), 0, X.rows(), 1) = -X.col(2);
  A.block(X.rows(), 2, X.rows(), 1) =  X.col(0);
  
  A.block(2*X.rows(), 0, X.rows(), 1) =  X.col(1);
  A.block(2*X.rows(), 1, X.rows(), 1) = -X.col(0);
  // add the "identity" block
  A.block(0,          3, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());
  A.block(X.rows(),   4, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());
  A.block(2*X.rows(), 5, X.rows(), 1) = Eigen::VectorXd::Ones(X.rows());

  // construct the [X_i - P_i] matrix:
  Eigen::MatrixXd XiPi(3*X.rows(), 1);
  XiPi.block(0,          0, X.rows(), 1) = X.col(0) - P.col(0);
  XiPi.block(X.rows(),   0, X.rows(), 1) = X.col(1) - P.col(1);
  XiPi.block(2*X.rows(), 0, X.rows(), 1) = X.col(2) - P.col(2);

  // optimal point u, solved using the colPivHouseholderQr solver: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  Eigen::VectorXd U;
  Eigen::MatrixXd ata, atxipi;

  ata = A.transpose()*A;
  atxipi = -1.0*A.transpose()*XiPi;
  U = ata.colPivHouseholderQr().solve(atxipi);

  // now reconstruct the transformation:
  t << U(3), U(4), U(5);

  Eigen::Matrix3d M;
  M << 1,   -U(2), U(1),
       U(2), 1,   -U(0),
      -U(1), U(0), 1;
  // the rotation comes from the rotation construction:
  closest_rotation(M, R);
}

