#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Get a list of the normals corresponding to all faces in FY
  Eigen::MatrixXd NY(FY.rows(), 3);
  igl::per_face_normals(VY, FY, NY);
  
  // Find the closest projection point on the mesh for each point in X
  double d;
  Eigen::RowVector3d p;
  for(int i = 0; i < X.rows(); i++) {
    
    // get initial values
    point_triangle_distance(X.row(i), VY.row(FY(0,0)), VY.row(FY(0,1)), VY.row(FY(0,2)), NY.row(0), d, p);
    D(i) = d;
    P.row(i) = p;
    N.row(i) = NY.row(0).normalized();
    
    // find better values
    for (int j = 1; j < FY.rows(); j++) {
      point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), NY.row(j), d, p);
      if (d < D(i)) {
        D(i) = d;
        P.row(i) = p;
        N.row(i) = NY.row(j).normalized();
      }
    }
  }
}
