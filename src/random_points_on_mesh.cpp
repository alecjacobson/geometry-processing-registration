#include "random_points_on_mesh.h"
#include <random>
#include <iostream>

using namespace std;

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n, 3);

  int num_faces = F.rows();
  std::uniform_int_distribution<int> random_face(0, num_faces);

  // Used to sample parallelogram triangle spans
  std::uniform_real_distribution<double> uniform_01(0.0, 1.0);

  // Three random generators (only start once) so we can repeat the experiment
  // One to choose a face, one for a (along side_1), one for b (along side_2)
  static std::default_random_engine generator_1;
  static std::default_random_engine generator_2;
  static std::default_random_engine generator_3;

  for (int i = 0; i < n; i++)
  {
    int sampled_face = random_face(generator_1);

    Eigen::VectorXd start_point = V.row(F(sampled_face, 0));
    Eigen::VectorXd side_1 = V.row(F(sampled_face, 1));
    Eigen::VectorXd side_2 = V.row(F(sampled_face, 2));

    side_1 = side_1 - start_point;
    side_2 = side_2 - start_point;
    // We will sample from the parallelogram formed with the points on the sample face
    double a = uniform_01(generator_2);
    double b = uniform_01(generator_3);    
    // Reflect back onto original triangle
    if (a + b > 1)
      a = 1 - a;
      b = 1 - b;

    Eigen::VectorXd sampled_point = start_point + a * side_1 + b * side_2;
    X.row(i) = sampled_point;
  }
}

