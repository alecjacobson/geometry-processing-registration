#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"

void point_to_point_rigid_matching(
        const Eigen::MatrixXd &X,
        const Eigen::MatrixXd &P,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {
//
    //Shift to the origin
    Eigen::MatrixXd X_ = X.rowwise() - X.colwise().mean();
    Eigen::MatrixXd P_ = P.rowwise() - P.colwise().mean();

    Eigen::MatrixXd M = (P_.transpose() * X_).transpose();
    closest_rotation(M, R);
    //compute t
    t = P.colwise().mean().transpose() - R * X.colwise().mean().transpose();

}

