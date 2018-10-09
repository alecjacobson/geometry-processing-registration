#include "random_points_on_mesh.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <iostream>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  srand(time(NULL));
  X.resize(n,3);
  Eigen::MatrixXd dblA(F.rows(), 1);
  igl::doublearea(V, F, dblA);
  Eigen::MatrixXd C(F.rows(), 1);
  igl::cumsum(dblA, 1, C);
  C = 1 / C(C.rows() - 1) * C;

  for (int i = 0; i < n; i++) {
	  // Generate random triangle index
	  double gamma = (double) rand() / (double) RAND_MAX;
	  // Binary search for the triangle index
	  int low = 0; int high = F.rows();
	  while (low < high) {
		  int m = (low + high) / 2;
		  if (C(m) > gamma) {
			  high = m;
		  }
		  else {
			  low = m + 1;
		  }
	  }
	  int picked = low;
	  // Generate
	  double alpha = (double) rand() / (double) RAND_MAX;
	  double beta = (double)rand() / (double)RAND_MAX;
	  if (alpha + beta > 1) {
		  alpha = 1 - alpha;
		  beta = 1 - beta;
	  }
	  Eigen::Vector3d x = V.row(F(picked, 0))
		  + alpha*(V.row(F(picked, 1)) - V.row(F(picked, 0)))
		  + beta*(V.row(F(picked, 2)) - V.row(F(picked, 0)));
	  X(i, 0) = x(0); X(i, 1) = x(1); X(i, 2) = x(2);
  }


}

