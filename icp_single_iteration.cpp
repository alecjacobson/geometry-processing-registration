#include "icp_single_iteration.h"
#include "random_points_on_mesh.h" 
#include "point_mesh_distance.h" 
#include "point_to_point_rigid_matching.h" 
#include "point_to_plane_rigid_matching.h" 

void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t) {
  
    // initialize R and t
    R = Eigen::Matrix3d::Identity();
    t = Eigen::RowVector3d::Zero();
    
    // retrieve a random sample of points on (VX, FX)
    Eigen::MatrixXd X;
    random_points_on_mesh(num_samples, VX, FX, X);
    
    // project X onto (VY, FY)
    Eigen::MatrixXd P;
    Eigen::MatrixXd N;
    Eigen::VectorXd D;
    point_mesh_distance(X, VY, FY, D, P, N);
    
    // update R and t in accordance with the given ICP method
    if (method == ICP_METHOD_POINT_TO_POINT) {
        point_to_point_rigid_matching(X, P, R, t);
    } else if (method == ICP_METHOD_POINT_TO_PLANE) {
        point_to_plane_rigid_matching(X, P, N, R, t);
    }
}
