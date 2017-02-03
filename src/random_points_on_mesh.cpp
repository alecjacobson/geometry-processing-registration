#include "random_points_on_mesh.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <random>

using namespace Eigen;

int binary_search(MatrixXd values, double val)
{
	int left = 0, right = values.rows() - 1, mid;
	
	if (val < values(0, 0))
		return 0;

	while (left < right)
	{
		mid = (left + right) / 2;
		if (values(mid, 0) < val)
			left = mid + 1;
		else
			right = mid;
	}

	return mid;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
	X.resize(n,3);
	MatrixXd dblArea;
	igl::doublearea(V, F, dblArea);
	MatrixXd areaCumsum;
	igl::cumsum(dblArea, 1, areaCumsum);

	int t;
	double sumVal;
	double alpha, beta;

	srand(time(NULL));

	for (int i = 0; i < X.rows(); ++i)
	{
		//choose index of triangle via binary search
		sumVal = (double(rand()) / RAND_MAX)*areaCumsum(F.rows()-1, 0);
		t = binary_search(areaCumsum, sumVal);

		//then, choose the point on triangle with index t
		alpha = double(rand()) / RAND_MAX;
		beta = double(rand()) / RAND_MAX;

		if (alpha + beta > 1)
		{
			alpha = 1 - alpha;
			beta = 1 - beta;
		}

		X.row(i) = V.row(F(t, 0)) + alpha * (V.row(F(t, 1)) - V.row(F(t, 0))) + beta * (V.row(F(t, 2)) - V.row(F(t, 0)));
	}
}

