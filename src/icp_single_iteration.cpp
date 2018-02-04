#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

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
    
    //Randomly sample points on the mesh
    Eigen::MatrixXd sampledPoints;
    random_points_on_mesh(num_samples,VX,FX, sampledPoints);
    
    Eigen::VectorXd D;
    Eigen::MatrixXd P, N;
    
    //Compute mesh distances
    point_mesh_distance(sampledPoints,VY,FY, D,P,N);
    
    //Run Point-to-Point or Point-to-Plane
    if (method == ICP_METHOD_POINT_TO_POINT) {
        point_to_point_rigid_matching(sampledPoints, P,R,t);
        
    }
    else if (method == ICP_METHOD_POINT_TO_PLANE){
        point_to_plane_rigid_matching(sampledPoints,P,N,R,t);
        
    }
    
    
}
