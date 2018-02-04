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
  // Replace with your code
    Eigen::MatrixXd XBar, PBar, M;
    M.resize(3,3);
    
    XBar.resizeLike(X);
    XBar = X;
    PBar.resizeLike(P);
    PBar = P;
    double xNum, pNum;
    xNum = X.rows();
    pNum = P.rows();
    
    //Compute the centralized query points and their closest points
    for (int i = 0; i < 3; i ++) {
        XBar.col(i).array() -= (XBar.col(i).mean());
        PBar.col(i).array() -= (PBar.col(i).mean());
    }
    
    //Estimate closed form of M
    M =   (XBar.transpose()) * PBar;
    
    //Project it to space of rotations
    closest_rotation(M,R);

    //Compute translation
    t = P.colwise().mean().transpose() - R * (X.colwise().mean().transpose());
    
}

