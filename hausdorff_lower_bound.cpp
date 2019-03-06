#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n) {
    
    // generate a sample of n points on the X mesh
    Eigen::MatrixXd X;
    random_points_on_mesh(n, VX, FX, X); 
    
    // distance, closest point, and normals to be populated by point_mesh_distance
    Eigen::VectorXd D;
    Eigen::MatrixXd P; 
    Eigen::MatrixXd N;
    
    // for each point in the sample, compute its distance to the Y mesh
    point_mesh_distance(X, VY, FY, D, P, N);
    
    // the lower bound on the hausdorff distance is the max in D
    return D.maxCoeff();
}
