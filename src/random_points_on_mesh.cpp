#include "random_points_on_mesh.h"
#include <random>
#include <iostream>

//#define VERTEX_ONLY

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  static std::default_random_engine generator;
#ifndef VERTEX_ONLY
  int max_face_index = F.rows();
  //from http://www.cplusplus.com/reference/random/uniform_int_distribution/
  std::uniform_int_distribution<int> rand_distribution_int(0, max_face_index);
  //from http://www.cplusplus.com/reference/random/uniform_real_distribution/
  std::uniform_real_distribution<double> rand_distribution_double(0.0, 1.0);

  for (int m = 0; m < X.rows(); m++)
  {
    int face_index = rand_distribution_int(generator);
    Eigen::MatrixXd v(3,3);
    //Randomly Fetch Triangles from the mesh
    //Use Barycentric coord to compute internal point within a triangle;
    //https://en.wikipedia.org/wiki/Barycentric_coordinate_system
    Eigen::RowVector3d lambda(3);
    for (int i = 0; i < 3; i++)
    {
      v.row(i) = V.row(F(face_index, i));
      lambda(i) = rand_distribution_double(generator);
    }
    lambda /= lambda.sum();
    X.row(m) = lambda*v;
  }
#else
  int max_vertex_index = V.rows();
  std::uniform_int_distribution<int> rand_distribution_int(0, max_vertex_index);
  for (int m = 0; m < X.rows(); m++)
  {
    int vertex_index = rand_distribution_int(generator);
    X.row(m) = V.row(vertex_index);
  }
#endif

}

