#include "icp_single_iteration.h"

void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    Eigen::MatrixXd X;
    random_points_on_mesh(num_samples,VX,FX,X);
    Eigen::VectorXd D;
    Eigen::MatrixXd P,N;
    point_mesh_distance(X,VY,FY,D,P,N);
    
    if(method == ICP_METHOD_POINT_TO_POINT){
        point_to_point_rigid_matching(X,P,R,t);
    } else {
        point_to_plane_rigid_matching(X,P,N,R,t);
    }
}
