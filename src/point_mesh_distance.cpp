#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <limits>
#include <Eigen/Dense>
// Compute the distances `D` between a set of given points `X` and their
// closest points `P` on a given mesh with vertex positions `VY` and face
// indices `FY`.
//
// Inputs:
//   X  #X by 3 list of query positions
//   VY  #VY by 3 list of mesh vertex positions
//   FY  #FY by 3 list of triangle mesh indices into VY
// Outputs:
//   D  #X list of distances from X to P 
//   P  #X by 3 list of 3d position of closest points
//   N  #X by 3 list of 3d unit normal vectors of closest points
/***************************************************************
//Compute the distances D between a set of given points X and their closest points P
//on a given mesh with vertex positions VY and face indices FY. For each point in P also
//output a corresponding normal in N.
*****************************************************************
//It is OK to assume that all points in P lie inside (rather than exactly at vertices
//or exactly along edges) for the purposes of normal computation in N.
******************************************************************/
void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  /*
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  for(int i = 0;i<X.rows();i++)
    P.row(i) = VY.row(i%VY.rows());
  D = (X-P).rowwise().norm();*/

  P.resizeLike(X);
  D.resize(X.rows());
  N.resizeLike(X);
  for (int i = 0; i < X.rows(); i++)
  {
    D(i) = std::numeric_limits<double>::max();
    int min_d_index = -1;
    for (int j = 0; j < FY.rows(); j++)
    {
      double d; Eigen::RowVector3d p;
      point_triangle_distance(X.row(i), VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), d, p);
      if (d < D(i))
      {
        P.row(i) = p;
        D(i) = d;
        min_d_index = j;
      }
    }
    Eigen::Vector3d edge1 = VY.row(FY(min_d_index, 1)) - VY.row(FY(min_d_index, 0));
    Eigen::Vector3d edge2 = VY.row(FY(min_d_index, 2)) - VY.row(FY(min_d_index, 0));
    N.row(i) = edge1.cross(edge2); 
    N.row(i) /= N.row(i).norm();
  }
}
