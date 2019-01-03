#include <random>
#include <map>
#include <chrono>

#include "random_points_on_mesh.h"
#include "igl/doublearea.h"
#include "igl/cumsum.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:

  // setup uniform distribution
  std::default_random_engine generator(
    std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  // compute face areas
  Eigen::VectorXd area;
  igl::doublearea(V, F, area);

  // compute cummulative sum
  Eigen::VectorXd cumsum;
  igl::cumsum(area, 1, cumsum);

  // store cumsum into std::map for efficient query
  std::map<double,int> cumsum_map;
  for (int i = 0; i < cumsum.rows(); ++i) {
    cumsum_map.insert(
      std::pair<double,int>(cumsum(i)/cumsum(cumsum.rows() - 1), i)
      );
  }

  // build output X
  double a, b;
  std::map<double,int>::iterator map_itr;
  X.resize(n,3);
  for(int i = 0; i < X.rows(); ++i) {
    // sample triangle
    a = distribution(generator);
    map_itr = cumsum_map.lower_bound(a);
    int& f = map_itr->second;

    // sample point on triangle
    a = distribution(generator);
    b = distribution(generator);
    X.row(i) = V.row(F(f,0)) + a*(V.row(F(f,1)) - V.row(F(f,0)))
        + b*( V.row(F(f,2)) - V.row(F(f,0)) );
  }


}

