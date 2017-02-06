#include "random_points_on_mesh.h"
#include "igl/cumsum.h"
#include "igl/doublearea.h"

using namespace igl;
using namespace Eigen;

void random_points_on_mesh(
	const int n,
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & F,
	Eigen::MatrixXd & X)
{
	X.resize(n, 3);

	VectorXd areas(F.rows());
	doublearea(V, F, areas);
	areas *= 0.5;

	VectorXd sums(F.rows());
	cumsum(areas, 1, sums);
	sums /= sums.maxCoeff();

	VectorXd gamma = VectorXd::Random(n);
	gamma = 0.5*(VectorXd::Ones(n) + gamma);

	MatrixXd alphaBeta = MatrixXd::Random(n, 2);
	alphaBeta = 0.5*(MatrixXd::Ones(n, 2) + alphaBeta);

	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < F.rows(); ++j)
		{
			if (sums(j) > gamma(i))
			{
				VectorXi indx = F.row(j);
				VectorXd v1 = V.row(indx(0)), v2 = V.row(indx(1)), v3 = V.row(indx(2));
				VectorXd ab = alphaBeta.row(i);
				double alpha = ab(0), beta = ab(1);
				if (alpha + beta > 1)
				{
					alpha = 1 - alpha;
					beta = 1 - beta;
				}

				VectorXd x = v1 + alpha*(v2 - v1) + beta*(v3 - v1);
				X.row(i) = x;

				break;
			}
		}
	}
}

