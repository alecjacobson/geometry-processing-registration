#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  Eigen::MatrixXd area;
  Eigen::MatrixXd cumsumA;
  igl::doublearea(V, F, area);
  igl::cumsum(area, 1, cumsumA);

  double ttl = cumsumA(cumsumA.rows() - 1, 0);
  for(int i=0;i<n;i++) {
  	double randx = ((double)rand()/(double)RAND_MAX) * ttl;
  	int idx = cumsumA.rows() / 2;
  	int gap = cumsumA.rows() / 4;
  	//go over boundary?
  	while(idx != 0 && idx != cumsumA.rows() - 1 && 
  		  cumsumA(idx - 1) < randx &&
  		  cumsumA(idx) > randx) {
  		if (randx > cumsumA(idx - 1)) {
  			idx += gap;
  		}
  		else {
  			idx -= gap;
  		}
  		gap /= 2;
  	}

  	double alpha = ((double)rand()/(double)RAND_MAX);
  	double beta = ((double)rand()/(double)RAND_MAX);

  	if (alpha + beta > 1) {
  		alpha = 1 - alpha;
  		beta = 1 - beta;
  	}

  	X.row(i) = V.row(F(idx, 0)) + 
  			   alpha * V.row(F(idx, 1)) +
  			   beta * V.row(F(idx, 2));
  }
}

