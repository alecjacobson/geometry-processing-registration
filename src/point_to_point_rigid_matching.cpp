#include "closest_rotation.h"
#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>

void point_to_point_rigid_matching(
    const Eigen::MatrixXd & X,
    const Eigen::MatrixXd & P,
    Eigen::Matrix3d & R,
    Eigen::RowVector3d & t)
{
    Eigen::Vector3d x_centroid = X.colwise().mean();
    Eigen::Vector3d p_centroid = P.colwise().mean();

    Eigen::MatrixXd X_bar = (X.rowwise() - x_centroid.transpose()).eval();
    Eigen::MatrixXd P_bar = (P.rowwise() - p_centroid.transpose()).eval();

    closest_rotation(P_bar.transpose() * X_bar, R);
    t = p_centroid - R * x_centroid;
}

