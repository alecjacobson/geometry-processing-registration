#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>

int firstGreater(Eigen::VectorXd &list, int first, int last, double tarVal) {
	int mid = first + (last - first) / 2;
	if (last == 0) {
		return 0;
	}
	else if (list(mid) > tarVal) {
		if (list(mid - 1) <= tarVal)
			return mid;
		else
			firstGreater(list, first, mid - 1, tarVal);
	}
	else {
		firstGreater(list, mid + 1, last, tarVal);
	}
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
	// Compute cumulative sum C of relative areas
	Eigen::VectorXd A(F.rows());	// Vector containing area for each triangle
	Eigen::VectorXd C(F.rows());  // Vector containing cumulative area	
	double gamma, alpha, beta;  // Random variables
	int ttidx;  //target triangle index - triangle to sample point from
	// Continuous uniform random sampling engine
	std::random_device rd;
	std::default_random_engine generator(rd());
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	
	// Compute cumulative sum of triangle area
	igl::doublearea(V, F, A);
	igl::cumsum(A/2.0, 1, C);
	C /= C(C.rows() - 1);  // Normalize cumulative sum

	X.resize(n, 3);
	for (int i = 0; i < X.rows(); i++) {
		// Area-weighted random sampling of triangles
		gamma = distribution(generator);
		ttidx = firstGreater(C, 0, C.size() - 1, gamma);

		// Uniform random point sampling
		alpha = distribution(generator);
		beta = distribution(generator);
		int v1 = F(ttidx, 0), v2 = F(ttidx, 1), v3 = F(ttidx, 2);
		if (alpha + beta > 1) {
			alpha = 1 - alpha;
			beta = 1 - beta;
		}
		X.row(i) = V.row(v1) + alpha*(V.row(v2) - V.row(v1)) + beta*(V.row(v3) - V.row(v1));
	}
}

