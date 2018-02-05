#include "random_points_on_mesh.h"

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

  Eigen::VectorXd A(F.rows());
  igl::doublearea(V, F, A);
  double A_x = A.sum();
  Eigen::VectorXd C(F.rows());
  igl::cumsum(A/A_x, 1, C);

  for(int i = 0; i < n; i++) {
  	double gamma = ((double) rand() / (RAND_MAX)) + 1;
  	// Pick a triangle
  	int tri_idx;
  	for(int j = 0; j < n; j++) { // TODO use binary search?
  		if(gamma <= C[j]) {
  			tri_idx = j;
  		}
  	}
  	// Pick a point
  	double alpha = ((double) rand() / (RAND_MAX)) + 1;
  	double beta = ((double) rand() / (RAND_MAX)) + 1;

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

