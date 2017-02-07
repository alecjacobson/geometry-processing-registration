#include "point_to_plane_rigid_matching.h"

// Given a set of source points X and corresponding target points P and their
// normals N, find the optimal rigid transformation (R,t) that aligns X to
// planes passing through P orthogonal to N, minimizing the point-to-point
// matching energy.
//
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    // Replace with your code
    //R = Eigen::Matrix3d::Identity();
    //t = Eigen::RowVector3d::Zero();
    
}
