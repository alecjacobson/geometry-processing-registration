#include "point_to_point_rigid_matching.h"
//#include <igl/polar_svd.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "closest_rotation.h"
// Given a set of source points X and corresponding target points P, find the
// optimal rigid transformation (R,t) that aligns X to P, minimizing the
// matching energy:
//
//   ¡¬ R X' - t' 1' - P' ¡¬^2
//
// Inputs:
//   X  #X by 3 set of source points
//   P  #X by 3 set of target points
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
//  
/****************************************************************
Given a set of source points X and corresponding target points P,
find the optimal rigid transformation(R, t) that aligns X to P,
minimizing the point - to - point matching energy.
*****************************************************************
You may implement either that ¡°Approximate¡± solution
via linearizing the rotation matrix or the ¡°closed form¡± solution
******************************************************************/
void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  using namespace Eigen;
  //Computer centroid
  Vector3d p_hat;
  Vector3d x_hat;
  for (int i = 0; i < 3; i++) 
  {
    p_hat(i) = P.col(i).sum() / P.rows();
    x_hat(i) = X.col(i).sum() / X.rows();
  }
  //Shift the geometry to the centroid
  MatrixXd X_bar = X;
  MatrixXd P_bar = P;
  for (int i = 0; i < 3; i++)
  {
    X_bar.col(i).array() -= x_hat(i);
    P_bar.col(i).array() -= p_hat(i);
  }

  Matrix3d M = X_bar.transpose()*P_bar;
  closest_rotation(M, R);
  t = p_hat - R*x_hat;
}

