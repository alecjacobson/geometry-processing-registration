#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <random>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  
  // Compute areas
  Eigen::MatrixXd A(V.rows(), 1);
  Eigen::MatrixXd cum_A(V.rows(), 1);
  igl::doublearea(V,F, A);
  igl::cumsum(A, 1, cum_A);
  cum_A = cum_A / A.sum();
  
  // Generate random numbers;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  for(int i = 0;i<X.rows();i++) {
	double alpha = distribution(generator);
        double beta = distribution(generator); 
        double gamma = distribution(generator);
        if (alpha + beta > 1) {
	    alpha = 1 - alpha;
	    beta = 1 - beta;
	}

 	// Linear search on cum_A to find the index of face;
	int idx = 0;
	while (cum_A(idx) < gamma) {
            idx++;
        }
        
        // Sample point from V(idx)
        Eigen::Vector3i Tri = F.row(idx);
        Eigen::Vector3d v = V.row(Tri(0));
        Eigen::Vector3d w = V.row(Tri(1)) - V.row(Tri(0));
        Eigen::Vector3d u = V.row(Tri(2)) - V.row(Tri(0));
        Eigen::Vector3d point = v + alpha * u + beta * w;
        X.row(i) = point;
  }
}

