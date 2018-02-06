#include "hausdorff_lower_bound.h"

#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"

using namespace Eigen;

double hausdorff_lower_bound(
		const Eigen::MatrixXd & VX,
		const Eigen::MatrixXi & FX,
		const Eigen::MatrixXd & VY,
		const Eigen::MatrixXi & FY,
		const int n) {
	
	// Sample points on the first mesh.
	MatrixXd xPoints;
	random_points_on_mesh(n, VX, FX, xPoints);

	// Compute distances.
	VectorXd distances;
	MatrixXd points;
	MatrixXd normals;
	point_mesh_distance(xPoints, VY, FY, distances, points, normals);

	double distance = 0;
	for (int i = 0; i < n; i++) {
		distance = std::max(distance, distances(i));
	}

	return distance;
}
