#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  Eigen::MatrixXd points_on_triangles(F.rows(),3);
  Eigen::MatrixXd dblA;
  Eigen::MatrixXd C;

  for (int i = 0; i < F.rows(); i++)
  {
		double alpha = ((double)rand() / (RAND_MAX));
		double beta = ((double)rand() / (RAND_MAX));
		if (alpha + beta > 1)
		{
			alpha = 1 - alpha;
			beta = 1 - beta;
		}

		int v1_index = F.row(i)(0);
		int v2_index = F.row(i)(1);
		int v3_index = F.row(i)(2);
		Eigen::RowVector3d x = V.row(v1_index) + alpha * (V.row(v2_index) -V.row(v1_index)) + 
												 beta * (V.row(v3_index) - V.row(v1_index));
		points_on_triangles.row(i) = x;
		

  }

  igl::doublearea(V, F, dblA);

  //std::cout << F.rows() << " is the # of faces" << std::endl;
  //std::cout << dblA.rows() << "," << dblA.cols() << std::endl;
  igl::cumsum(dblA, 1, C);

  //std::cout << C(C.rows()-1) << std::endl;
  double area_t = C(C.rows() - 1);
  C = C / area_t;
  //std::cout << C(C.rows() - 1) << std::endl;

  std::function<int(int, int, double)> search;
  search = [&C, &search](int l, int r, double gamma)->int
  { 
	  int mid = l + (r - l) / 2;
	  if (r >= l)
	  {
		  if (C(mid) == gamma)
			  return mid;

		  if (C(mid) > gamma)
			  return search(l, mid - 1, gamma);

		  return search(mid + 1, r, gamma);
	  }

	  return mid+1;
  
  };

  X.resize(n,3);
  for (int i = 0; i < X.rows(); i++)
  {
	  double gamma = ((double)rand() / (RAND_MAX));
	  int index = search(0, C.rows(), gamma);
	  X.row(i) = points_on_triangles.row(index);
  }
}

