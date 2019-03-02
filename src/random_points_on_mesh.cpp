#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>


int binary_find(Eigen::VectorXd lst, double val) {
	int idx = 0;
	if (lst.rows() <= 1) return idx;

	int mid = lst.rows()/2;
	if (lst(mid) < val) {
		idx = mid + binary_find(lst.segment(mid, lst.rows()), val);
	} else {
		idx = binary_find(lst.head(mid), val);
	}
	return idx;
}


void random_points_on_mesh(
	const int n,
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & F,
	Eigen::MatrixXd & X)
{
	// calculate sumulative sum
	Eigen::VectorXd dblx, csum;
	igl::doublearea(V, F, dblx);
	igl::cumsum(dblx, 1, csum);
	double Ax = csum(csum.rows() - 1);
	csum /= Ax;

	// uniform distribution generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);

	X.resize(n, 3);
	for (int i = 0; i < X.rows(); i++) {
		// find random index idx
		double gamma = dis(gen);
		int idx = binary_find(csum, gamma);

		// generate and configurate alpha, beta
	    double alpha = dis(gen);
	    double beta = dis(gen);
	    if (alpha + beta > 1) {
	    	alpha = 1 - alpha;
	    	beta = 1 - beta;
	    }

	    Eigen::VectorXd v1 = V.row(F(idx, 0));
	    Eigen::VectorXd v2 = V.row(F(idx, 1));
	    Eigen::VectorXd v3 = V.row(F(idx, 2));

		X.row(i) = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
	}
}
