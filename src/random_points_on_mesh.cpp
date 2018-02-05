#include "random_points_on_mesh.h"

#include <iostream>
#include <random>

#include <igl/doublearea.h>
#include <igl/cumsum.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  Eigen::VectorXd A(F.rows());
  igl::doublearea(V, F, A);
  A = A / 2.0;
  double A_x = A.sum();
  Eigen::VectorXd C(F.rows());
  igl::cumsum(A/A_x, 1, C);
  // std::cout << "C: " << C << std::endl;
  for(int i = 0; i < n; i++) {
  	double gamma = distribution(generator);
  	// Pick a triangle
  	int tri_idx = 0;
  	for(int j = 0; j < C.size(); j++) { // TODO use binary search?
  		if(C[j] > gamma) {
  			tri_idx = j;
            break;
  		}
    }
    // std::cout<< "gamma: " << gamma << std::endl;
    // std::cout<< "index: " << tri_idx << std::endl;
  	// Pick a point
  	double alpha = distribution(generator);
  	double beta = distribution(generator);

  	if(alpha + beta > 1) {
  		alpha = 1.0 - alpha;
  		beta = 1.0 - beta;
  	}

  	Eigen::RowVector3d v1 = V.row(F(tri_idx, 0));
  	Eigen::RowVector3d v2 = V.row(F(tri_idx, 1));
  	Eigen::RowVector3d v3 = V.row(F(tri_idx, 2));

  	X.row(i) = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
  }
}

