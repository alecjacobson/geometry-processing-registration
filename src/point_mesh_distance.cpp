#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <iostream>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // // Replace with your code
  // P.resizeLike(X);
  // N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  // for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  // D = (X-P).rowwise().norm();

  Eigen::RowVector3d a, b, c, p;
  a.resize(3); b.resize(3); c.resize(3); p.resize(3);
  double d;

  P.resize(X.rows(),3);
  N.resize(X.rows(),3);
  D.resize(X.rows());

  // Compute all face normals for later use
  Eigen::MatrixXd allN;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), allN);

  // First, loop through each point in X
  for (int ii = 0; ii < X.rows(); ii++)
  {
    // Extract the current x-point
    Eigen::RowVector3d x; x(0) = X(ii,0); x(1) = X(ii,1); x(2) = X(ii,2);

    // Now loop through all the triangles in Y and compute the point-to-triangle 
    // closest points. The smallest of these distances is the one we're after.
    double x_min_dist = 1.0e10;

    for (int jj = 0; jj < VY.rows(); jj++)
    {
        a(0) = VY(FY(jj,0),0); a(1) = VY(FY(jj,0),1); a(2) = VY(FY(jj,0),2);
        b(0) = VY(FY(jj,1),0); a(1) = VY(FY(jj,1),1); a(2) = VY(FY(jj,1),2);
        c(0) = VY(FY(jj,2),0); a(1) = VY(FY(jj,2),1); a(2) = VY(FY(jj,2),2);

        point_triangle_distance(x, a, b, c, d, p);

        if (d < x_min_dist)
        {
          x_min_dist = d;

          // Pick this closest point to be *the* closest point
          P(ii,0) = p(0);
          P(ii,1) = p(1);
          P(ii,2) = p(2);

          // Pick the right normal that corresponds to this closest triangle
          N(ii,0) = allN(jj,0);
          N(ii,1) = allN(jj,1);
          N(ii,2) = allN(jj,2);
        }

    }

    D(ii) = x_min_dist;

  }


  return;

}
