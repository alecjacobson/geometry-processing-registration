#include "icp_single_iteration.h"
#include "closest_rotation.h"
#include "point_mesh_distance.h"
#include "random_points_on_mesh.h"
#include "hausdorff_lower_bound.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include <Eigen/Core>
#include <Eigen/Dense>
// Conduct a single iteration of the iterative closest point method align (VX,FX)
// to (VY,FY) by finding the rigid transformation (R,t) minimizing the matching
// energy.
//
// The caller can specify the number of samples num_samples used to approximate
// the integral over X and specify the method (point-to-point or point-to-plane).
//
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
    // Replace with your code
    R = Eigen::Matrix3d::Identity();
    t = Eigen::RowVector3d::Zero();

    // Point to Point first...

    // sample mesh
    Eigen::MatrixXd X,P;
    random_points_on_mesh( num_samples, VX, FX, X );

    // project all X onto target mesh
    Eigen::VectorXd D;
    Eigen::MatrixXd N;
    point_mesh_distance( X, VY, FY, D, P, N );
  
    // udpate rigid transofrm to best match X and P
    point_to_point_rigid_matching( X, P, R, t );

    // looks like this is already done in our calling routine...
    // transform original source mesh by R and T
    //VX = ((VX*R).rowwise() + t).eval();
    //VX = (VX*R);
    //VX = VX.rowwise() + t;
}


/*
    // small rotation to test...
    double ct = std::cos( 0.1 );
    double st = std::sin( 0.1 );

    Eigen::Matrix3d r;
    r << 1,   0,   0,
         0,   ct,   -st,
         0,   st,   ct;
    r.setIdentity(); //uh, let's remove the rotation...just test trans
    Eigen::RowVector3d t2( 0, 1, 0 );
    P = (X * R).rowwise() + t2;
*/