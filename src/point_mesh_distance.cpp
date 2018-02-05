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
  int num_points = X.rows();

  // Reshape the vector of distances
  D = Eigen::VectorXd(num_points);

  // We shall construct a matrix of nearest points and normals at those points
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(num_points, 3);

  // YAY! Already implemented
  Eigen::MatrixXd allNormals;
  igl::per_face_normals(VY, FY, allNormals);
    
  for(int i = 0; i < num_points; i++)
  {
    // Track best distance, face index and point within that face
    double d_min;
    int f_min;
    Eigen::RowVector3d p_min;

    Eigen::RowVector3d x = X.row(i);
    for(int f = 0; f < FY.rows(); f++)
    {
      int a_index = FY(f, 0);
      int b_index = FY(f, 1);
      int c_index = FY(f, 2);
      Eigen::RowVector3d a = VY.row(a_index);
      Eigen::RowVector3d b = VY.row(b_index);
      Eigen::RowVector3d c = VY.row(c_index);

      double d;
      Eigen::RowVector3d p;
      point_triangle_distance(x, a, b, c, d, p);

      if (f == 0){
        d_min = d;
        f_min = f;
        p_min = p;
      }

      else if(d < d_min){
          d_min = d;
          f_min = f;
          p_min = p;
      }
    }

    // Update P, D, N
    P.row(i) = p_min;
    D(i) = d_min;
    N.row(i) = allNormals.row(f_min);
  }
}
