#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n,3);

  // first get all of the areas of the triangles:
  Eigen::VectorXd area;
  igl::doublearea(V, F, area);
  // need the C matrix to be normalized [0,1]
  // area /= 2.0; // <-not needed
  area /= area.sum();
  // cumulative sum of normalized areas
  Eigen::VectorXd C;
  igl::cumsum(area, 1, C);

  // uniform random number generator:
  // to call a random number use dist(gen);
  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  // get the n-samples wanted
  for(int32_t i = 0; i < n; i++)
  {
    double alpha = dist(gen);
    double beta  = dist(gen);
    double gamma = dist(gen);

    // make sure that the alpha and beta are within the limits of the triangle:
    if( alpha + beta > 1.0 )
    {
      alpha = 1 - alpha;
      beta  = 1 - beta;      
    }

    // now find the correct C_i from the C array using a small binary search
    int32_t middle = C.rows() / 2;
    int32_t left   = 0;
    int32_t right  = C.rows() - 1;
    while( left < right )
    {
      middle = (left + right) / 2;
      if( C(middle, 0) >= gamma )
	right = middle;
      else
	left = middle + 1;
    }

    // now get the random point:
    // x = v₁ + α(v₂ - v₁) + β(v₃ - v₁)
    X.row(i) = V.row( F(middle, 0) ) + alpha*( V.row(F(middle, 1)) - V.row(F(middle, 0)) ) + beta*( V.row(F(middle, 2)) - V.row(F(middle, 0)) );
  }
}

