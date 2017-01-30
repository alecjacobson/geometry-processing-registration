#include "random_points_on_mesh.h"
#include <algorithm>
#include <random>
#include "igl/cumsum.h"
#include "igl/doublearea.h"



void random_points_on_mesh(
	const int n,
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & F,
	Eigen::MatrixXd & X)
{

	//First compute the cumulative sums to make it easier going forward.
	Eigen::VectorXd face_areas;
	igl::doublearea(V, F, face_areas);
	auto Ax = face_areas.sum();
	Eigen::VectorXd C;
	face_areas /= Ax; //Since we're normalizing here, the double area gets cancelled out.
	igl::cumsum(face_areas, 1, C);

	X.resize(n, 3);

	std::random_device rd;
	std::mt19937_64 gen(rd());

	//For each sample, first compute a triangle index, then compute a random point in that triangle.
	for (int i = 0; i < n; i++) {
		auto gamma = std::generate_canonical<double, 64>(gen);
		auto alpha = std::generate_canonical<double, 64>(gen);
		auto beta = std::generate_canonical<double, 64>(gen);

		//Do a binary search on C for the first number less than gamma.
		auto triangle_index = std::distance(C.data(),
											std::upper_bound(C.data(),
															 C.data() + C.size(),
															 gamma));
		if (alpha + beta > 1.0) {
			alpha = 1.0 - alpha;
			beta = 1.0 - beta;
		}

		auto& v1 = V.row(F(triangle_index, 0));
		auto& v2 = V.row(F(triangle_index, 1));
		auto& v3 = V.row(F(triangle_index, 2));

		X.row(i) = v1 + alpha*(v2 - v1) + beta*(v3 - v1);
	}
}

