#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <cfloat>
#include <Eigen/dense>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());

  // loop through each point:
  for( int32_t i = 0; i < X.rows(); i++)
  {
    double distMin = DBL_MAX;
    Eigen::RowVector3d pointMin;
    Eigen::VectorXi indexMin;

    // loop through each triangle
    for( int32_t j = 0; j < FY.rows(); j++)
    {
      double distance = 0;
      Eigen::RowVector3d pointOnTriangle;
      Eigen::VectorXi indices = FY.row(j);

      Eigen::RowVector3d p0 = VY.row(indices(0));
      Eigen::RowVector3d p1 = VY.row(indices(1));
      Eigen::RowVector3d p2 = VY.row(indices(2));

      point_triangle_distance(X.row(i), p0, p1, p2, distance, pointOnTriangle);

      if( distance < distMin )
      {
	distMin  = distance;
	pointMin = pointOnTriangle;
	indexMin = indices;
      }
    }

    // now we have the closest point on the surface and the corresponding triangle to the point on X
    // add this point to P, the normal to N, and the distance into D:
    P.row(i) = pointMin;
    D(i) = distMin;
    // cross product for the indices:
    Eigen::RowVector3d p0 = VY.row(indexMin(0));
    Eigen::RowVector3d p1 = VY.row(indexMin(1));
    Eigen::RowVector3d p2 = VY.row(indexMin(2));

    Eigen::RowVector3d edge0 = p1-p0;
    Eigen::RowVector3d edge1 = p2-p0;
    Eigen::RowVector3d n = edge0.cross(edge1);
    n.normalize();    
    N.row(i) = n;
  }  
}
