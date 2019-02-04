#include "closest_rotation.h"
#include "point_to_plane_rigid_matching.h"
#include <Eigen/SVD>

void point_to_plane_rigid_matching(
    const Eigen::MatrixXd & X,
    const Eigen::MatrixXd & P,
    const Eigen::MatrixXd & N,
    Eigen::Matrix3d & R,
    Eigen::RowVector3d & t)
{
    Eigen::MatrixXd XN(X.rows(), 3);
    for (int i = 0; i < X.rows(); i++) 
    {
        Eigen::RowVector3d x = X.row(i);
        Eigen::RowVector3d n = N.row(i);
        XN.row(i) = x.cross(n);
    }

    Eigen::MatrixXd A(P.rows(), 6);
    A << XN, N;

    Eigen::VectorXd b(X.rows());
    for (int i = 0; i < X.rows(); i++) {
        b[i] = -(X.row(i) - P.row(i)).dot(N.row(i));
    }

    Eigen::VectorXd u = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix3d U;

    // first three elements of u define the angles of rotation
    // about the x, y, and z axis, in that order
    U << Eigen::RowVector3d(0, -u[2], u[1]),
        Eigen::RowVector3d(u[2], 0, -u[0]),
        Eigen::RowVector3d(-u[1], u[0], 0);

    Eigen::MatrixXd M = Eigen::Matrix3d::Identity() + U;
    closest_rotation(M, R);

    // last three elements of u define the translation
    t << u[3], u[4], u[5];
}
