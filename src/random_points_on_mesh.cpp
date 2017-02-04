#include "random_points_on_mesh.h"
#include "igl/doublearea.h"
#include "igl/cumsum.h"
#include <random>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
	// Define standard uniform R.V sampler
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dist(0, 1);

	// Compute cumsum
	Eigen::VectorXd dblA;
	Eigen::VectorXd C;
	igl::doublearea(V, F, dblA);
	igl::cumsum(dblA, 1, C);

	C = C / C.maxCoeff();

	X.resize(n,3);
	for (int i = 0; i < X.rows(); i++) {
	 
		// Generate 1 sample from standard uniform R.V.
		double gamma = dist(gen);

		// Randomly select triangle to sample from
		// Feels like Eigen should have a nicer way to do this
		int FIndex = 0;
		for (int j = 0; j < C.rows(); ++j) {
			if (C(j) > gamma) {
				FIndex = j;
				break;
			}
		}
		// Select verticies from V
		Eigen::VectorXi VIndex = F.row(FIndex);
		Eigen::VectorXd v0 = V.row(VIndex(0));
		Eigen::VectorXd v1 = V.row(VIndex(1));
		Eigen::VectorXd v2 = V.row(VIndex(2));

		// Generate 2 samples from the standard uniform R.V.
		double alpha = dist(gen);
		double beta = dist(gen);
		if (alpha + beta > 1) {
			alpha = 1 - alpha;
			beta = 1 - beta;
		}

		// Compute point sample from triangle
		X.row(i) = v0 + alpha*(v1 - v0) + beta*(v2 - v1);
	}
}

