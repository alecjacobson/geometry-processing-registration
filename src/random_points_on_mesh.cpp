#include "random_points_on_mesh.h"

#include "igl/doublearea.h"
#include "igl/cumsum.h"

inline float randomfloat() {
	float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	return r;
}

void random_points_on_mesh(const int n, const Eigen::MatrixXd & V,
		const Eigen::MatrixXi & F, Eigen::MatrixXd & X) {
	// REPLACE WITH YOUR CODE:
	X.resize(n, 3);

	// faster
	int vnum = V.rows();
	for (int i = 0; i < n; i++) {
		// find where it is
		int idx = int(randomfloat() * vnum);
		X.row(i) = V.row(idx);
	}
	return;

	// first, calculate the area of each triangle
	Eigen::MatrixXd areas;
	igl::doublearea(V, F, areas);

	// second, calculate the cumsum of the points
	Eigen::MatrixXd consumareas;
	igl::cumsum(areas, 1, consumareas);

	// for each random
	int fnum = F.rows();
	for (int i = 0; i < n; i++) {
		float tmp = randomfloat() * consumareas(fnum - 1, 0);

		// find where it is
		int idx = 0;
		for (int j = 0; j < fnum; j++) {
			if (tmp <= consumareas(j, 0)) {
				idx = j;
				break;
			}
		}

		double a1 = randomfloat();
		double a2 = (1 - a1) * randomfloat();
		double a3 = 1 - a1 - a2;

		X.row(i) = a1 * V.row(F(idx, 0)) + a2 * V.row(F(idx, 1))
				+ a3 * V.row(F(idx, 2));
	}
}

