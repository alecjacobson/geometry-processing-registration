#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
    //Start by sampling points
    Eigen::MatrixXd sampledPoints;
    random_points_on_mesh(n,VX,FX, sampledPoints);
    
    Eigen::VectorXd D;
    Eigen::MatrixXd P, N;
    
    point_mesh_distance(sampledPoints,VY,FY, D,P,N);
    //Then run point_mesh_distance
    
    return D.maxCoeff();
}
