#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void createA(const Eigen::MatrixXd &X, Eigen::MatrixXd &A) {
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(X.rows());
    Eigen::VectorXd one = Eigen::VectorXd::Ones(X.rows());
//-
    Eigen::VectorXd X_1 = X.col(1);
    Eigen::VectorXd X_2 = X.col(2);
    Eigen::VectorXd X_0 = X.col(0);

    A << zero, X_2, -X_1, one, zero, zero,
            -X_2, zero, X_0, zero, one, zero,
            X_1, -X_0, zero, zero, zero, one;
}

Eigen::MatrixXd diagN(const Eigen::MatrixXd &N, int idx) {
    return N.col(idx).asDiagonal();
}

void point_to_plane_rigid_matching(
        const Eigen::MatrixXd &X,
        const Eigen::MatrixXd &P,
        const Eigen::MatrixXd &N,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {

    Eigen::MatrixXd A(3 * X.rows(), 6);
    Eigen::MatrixXd d_N(X.rows(), 3 * X.rows());

    //Creating A, diagonal_N and X-P
    createA(X, A);
    d_N << diagN(N, 0), diagN(N, 1), diagN(N, 2);
    Eigen::MatrixXd XP = X - P;

    Eigen::VectorXd B = Eigen::VectorXd::Zero(X.rows() * 3);
    B << (X - P).col(0), (X - P).col(1), (X - P).col(2);

    //Solving the equation (52) in Readme.
    A = d_N * A;
    B = d_N * B;

    //need to be defined dynamically so that JacobiSVD works.
    Eigen::VectorXd u;


    u = (A.transpose() * A).inverse() * (-A.transpose() * B);

    double alpha = u[0];
    double beta = u[1];
    double gamma = u[2];

    Eigen::Matrix3d M = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix3d buff;
    buff << 0, -gamma, beta,
            gamma, 0, -alpha,
            -beta, alpha, 0;

    M = M - buff;
    closest_rotation(M, R);
    t << u(3), u(4), u(5);


}
