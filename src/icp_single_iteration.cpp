#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"
#include "point_mesh_distance.h"

void icp_single_iteration(
        const Eigen::MatrixXd &VX,
        const Eigen::MatrixXi &FX,
        const Eigen::MatrixXd &VY,
        const Eigen::MatrixXi &FY,
        const int num_samples,
        const ICPMethod method,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {

    /*
     *  Algorithm:
            X ← sample source mesh (V_X,F_X)
            P0 ← project all X onto target mesh (V_Y,F_Y)
            R,t ← update rigid transform to best match X and P0
            V_X ← rigidly transform original source mesh by R and t
     */

    Eigen::MatrixXd X, P, N;
    Eigen::VectorXd D;
    random_points_on_mesh(num_samples, VX, FX, X);
    point_mesh_distance(X, VY, FY, D, P, N);

    switch (method) {
        case ICPMethod::ICP_METHOD_POINT_TO_PLANE:
            point_to_plane_rigid_matching(X, P, N, R, t);
            break;
        case ICPMethod::ICP_METHOD_POINT_TO_POINT:
            point_to_point_rigid_matching(X, P, R, t);
            break;
        default:
            assert(true);
    }


}
