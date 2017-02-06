#include "random_points_on_mesh.h"

#include <igl/cumsum.h>
#include <igl/doublearea.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  srand(time(NULL));

  //Calculate our area-weighted sampling data
  Eigen::VectorXd triangle_areas(F.rows());
  igl::doublearea(V, F, triangle_areas);  
  triangle_areas = triangle_areas / triangle_areas.sum();

  Eigen::VectorXd cummulative_area(F.rows());
  igl::cumsum(triangle_areas, 1, cummulative_area);  

  for (int i = 0; i < X.rows(); i++) {

	  //Pick an area-weighted random triangle index
	  double gamma = ((double)rand() / (RAND_MAX)); //Uniform random between 0 and 1
	  int index = 0;
	  while (cummulative_area(index) < gamma && index < F.rows()) { index++; }

	  //Pick uniformly distributed barycentric coordinates on a triangle
	  double alpha = ((double)rand() / (RAND_MAX));
	  double beta = ((double)rand() / (RAND_MAX));
	  if (alpha + beta > 1) { //Ensure the coordinates are within the original triangle
		  alpha = 1 - alpha;
		  beta = 1 - beta;
	  }

	  //Set the selected random point
	  auto p0 = V.row(F(index, 0));
	  auto p1 = V.row(F(index, 1)) - p0;
	  auto p2 = V.row(F(index, 2)) - p0;
	  X.row(i) = p0 + (alpha * p1) + (beta * p2);
  }
}

