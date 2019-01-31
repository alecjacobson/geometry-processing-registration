#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "hausdorff_lower_bound.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include <iostream>

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

  // Set seed for random numbers
  srand(1);

  // Initial guesses
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd VX_updated = VX;

  // Define tolerance based on initial Haussdorf distance
  double H = hausdorff_lower_bound(VX_updated, FX, VY, FY, num_samples);
  double tol = 1.0e-3;
 
  int iter = 0;
  while (H > tol && iter < 10)
  {
    iter++;

    H = hausdorff_lower_bound(VX_updated, FX, VY, FY, num_samples);
    std::cout << "Haussdorf lower bound: " << H << " at iteration " << iter << std::endl;

    // Sample the source mesh
    Eigen::MatrixXd X;
    random_points_on_mesh(num_samples, VX_updated, FX, X);

    // Compute the set of closest points for all X samples
    Eigen::VectorXd D;
    Eigen::MatrixXd P, N;
    point_mesh_distance(X, VY, FY, D, P, N);

    if (method == ICP_METHOD_POINT_TO_POINT)
    {
      point_to_point_rigid_matching(X, P, R, t);

      // Apply the updated transforms
      for (int ii = 0; ii < VX_updated.rows(); ii++)
      {
        Eigen::MatrixXd temp1 = R*(VX_updated.row(ii).transpose()) + t.transpose();
        VX_updated(ii,0) = temp1(0,0);
        VX_updated(ii,1) = temp1(1,0);
        VX_updated(ii,2) = temp1(2,0);
      }

    }
    else
    {
      point_to_plane_rigid_matching(X, P, N, R, t);

      // Apply the updated transforms
      for (int ii = 0; ii < VX_updated.rows(); ii++)
      {
        Eigen::MatrixXd temp1 = R*(VX_updated.row(ii).transpose()) + t.transpose();
        VX_updated(ii,0) = temp1(0,0);
        VX_updated(ii,1) = temp1(1,0);
        VX_updated(ii,2) = temp1(2,0);
      }

    }


  }


}
