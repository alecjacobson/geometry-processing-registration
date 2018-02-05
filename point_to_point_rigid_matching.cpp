#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t) {
    
    // compute centroids for P and X
    Eigen::RowVector3d xc;
    xc(0) = X.col(0).sum() / X.rows();
    xc(1) = X.col(1).sum() / X.rows();
    xc(2) = X.col(2).sum() / X.rows();
    
    Eigen::RowVector3d pc;
    pc(0) = P.col(0).sum() / P.rows();
    pc(1) = P.col(1).sum() / P.rows();
    pc(2) = P.col(2).sum() / P.rows();

    // compute coordinates for the points in X and P relative to their centroids
    Eigen::MatrixXd XB;
    Eigen::MatrixXd PB;
    
    XB.resizeLike(X);
    PB.resizeLike(P);
    
    for (int i = 0; i < X.rows(); i++) {
        XB.row(i) = X.row(i) - xc;
    }
    
    for (int j = 0; j < P.rows(); j++) {
        PB.row(j) = P.row(j) - pc;
    }
    
    // compute M using the relative coordinates
    Eigen::Matrix3d M = PB.transpose()*XB;

    // find the optimal R by passing M to closest_rotation and use it
    // to compute the optimal t
    closest_rotation(M, R);
    t = pc - (R*xc.transpose()).transpose();
    
}

