#include "hausdorff_lower_bound.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

#include <iostream>
#include <assert.h>
// Compute a lower bound on the directed Hausdorff distance from a given mesh
// (VX,FX) to another mesh (VY,FY). This function should be implemented by
// randomly sampling the X mesh.
// X --> Y
double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
    static bool debug( true );
    
    double hausdorff( -1 );
    // Replace with your code
    // assuming n is the # of samples for the distance

    Eigen::MatrixXd X; // random points on VX
    random_points_on_mesh( n, VX, FX, X );

    Eigen::MatrixXd P;
    Eigen::VectorXd D;
    Eigen::MatrixXd N;
    point_mesh_distance( X, VY, FY, D, P, N );
    hausdorff = D.maxCoeff();
    std::cout << "Calculated hausdorff as " << hausdorff << std::endl;
    
    assert( hausdorff > -1 );
    return hausdorff;
}
