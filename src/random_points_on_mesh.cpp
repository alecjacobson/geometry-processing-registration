#include "random_points_on_mesh.h"
#include <random>
#include <algorithm>
#include "igl/doublearea.h"
#include "igl/cumsum.h"

using namespace Eigen;

// Random number generator instance.
std::minstd_rand generator;

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
	X.resize(n,3);

	int triangleCount = F.rows();
	int vertexCount = V.rows();
	int sampleCount = X.rows();

	Eigen::MatrixXd doubleAreas;
	doubleAreas.resize(triangleCount, 1);
	igl::doublearea(V, F, doubleAreas);

	// Populate listing of cumulative triangle areas. We also divide so that
	// it is out of one and can be sampled from later on.
	Eigen::MatrixXd cumSumAreas;
	igl::cumsum(doubleAreas, 1, cumSumAreas);
	cumSumAreas /= cumSumAreas(cumSumAreas.rows() - 1, 0);

	std::vector<double> cumulativeAreas;
	for (int i = 0; i < cumSumAreas.rows(); i++) {
		cumulativeAreas.push_back(cumSumAreas(i, 0));
	}

	// Generate the random points.
	for (int i = 0; i < sampleCount; i++) {
		std::uniform_real_distribution<double> distribution(0, 1);
		double sample1D = distribution(generator);

		// This employs binary search to convert the 1D sample into an actual
		// triangle index.
		auto comparator = [](double a, double b) -> bool { return a < b; };
		auto location = std::lower_bound(cumulativeAreas.begin(), 
				cumulativeAreas.end(), sample1D, comparator);
		auto it = location - cumulativeAreas.begin();
		int triangleIndex = it;

		// Now we just have to sample within the triangle. We do the 
		// parallelogram thing to create good sample parameters.
		double alpha = distribution(generator);
		double beta = distribution(generator);
		if (alpha + beta > 1) {
			alpha = 1 - alpha;
			beta = 1 - beta;
		}

		// Now we just interpolate. The sample parameters are restricted
		// already to form a triangle.
		Vector3d p1 = V.row(F(triangleIndex, 0));
		Vector3d p2 = V.row(F(triangleIndex, 1));
		Vector3d p3 = V.row(F(triangleIndex, 2));
		Vector3d position = p1 + (p2 - p1) * alpha + (p3 - p1) * beta;
		X.row(i) = position;
	}

}

