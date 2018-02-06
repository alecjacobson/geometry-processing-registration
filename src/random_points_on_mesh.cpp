#include "random_points_on_mesh.h"
#include <iostream>
#include <random>

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

    static std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.,ttl);
    double randx = distribution(generator);

  	int idx = cumsumA.rows() / 2;
  	int gap = cumsumA.rows() / 4;
  	//go over boundary?
  	while(idx >= 1 && idx <= cumsumA.rows() - 1 && gap != 0) {
      if (cumsumA(idx - 1, 0) < randx && cumsumA(idx, 0) > randx) {
        // std::cout << "hihiiiihi" << std::endl;
        break;
      }
  		if (randx > cumsumA(idx)) {
  			idx += gap;
  		}
  		else {
  			idx -= gap;
  		}
  		gap = gap / 2;
      if (gap == 0) {
        gap = 1;
      }
      // std::cout << "==============" << std::endl;
      // std::cout << cumsumA(idx - 1) << std::endl;
      // std::cout << randx << std::endl;
      // std::cout << cumsumA(idx) << std::endl;
      // std::cout << gap << std::endl;
      // std::cout << idx << std::endl;
      // std::string mystr;
      // getline (std::cin, mystr);

  	}
    // std::cout << idx << std::endl;
    std::uniform_real_distribution<double> distribution_alpha(0.,1.);
    std::uniform_real_distribution<double> distribution_beta(0.,1.);
  	double alpha = distribution_alpha(generator);
  	double beta = distribution_beta(generator);

  	if (alpha + beta > 1) {
  		alpha = 1 - alpha;
  		beta = 1 - beta;
  	}

    if (idx < 0) {
      idx = 0;
    }
    if (idx > cumsumA.rows() - 1) {
      idx = cumsumA.rows() - 1;
    }

  	X.row(i) = V.row(F(idx, 0)) + 
  			   alpha * (V.row(F(idx, 1)) - V.row(F(idx, 0))) +
  			   beta * (V.row(F(idx, 2)) - V.row(F(idx, 0)));
    // std::cout << "==============" << std::endl;
    // std::cout << idx << std::endl;
    // std::cout << randx << std::endl;
    // std::cout << alpha << std::endl;
    // std::cout << beta << std::endl;
  }
}

