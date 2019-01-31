#include "random_points_on_mesh.h"
#include "stdlib.h"
#include <iostream>
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include "point_triangle_distance.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // // REPLACE WITH YOUR CODE:
  // X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

  	Eigen::MatrixXd dblA, Ci;

  	// Compute triangle areas
  	igl::doublearea(V, F, dblA);

  	// Convert the areas to relative areas by dividing by total mesh area
  	double total_area = dblA.sum();
  	for (int ii = 0; ii < dblA.rows(); ii++)
  		dblA(ii) = dblA(ii)/total_area;

  	// Cumulative sum for picking triangles
  	igl::cumsum(dblA, 1, Ci);

  	// std::cout << Ci(Ci.rows()-1) << std::endl;
  	// std::cout << total_area << ", " << Ci(Ci.rows()-1) << std::endl;
  	
   	// Set the size of the output matrix, which is a list a points
  	X.resize(n,3);
  	// Generate n random points on the mesh
  	for (int jj = 0; jj < n; jj++)
  	{
		// Define the random constants as per notes
		double a = -((double)rand()/(double)(RAND_MAX + 1));
		double b = -((double)rand()/(double)(RAND_MAX + 1));

		if (a + b > 1.0)
		{
			a = 1.0 - a;
			b = 1.0 - b;
		}

  		double c = -((double)rand()/(double)(RAND_MAX + 1));

  		int flag = 0, kk = 0;
  		while (flag == 0)
  		{
  			if (Ci(kk) > c)
  			{
  				flag = 1;
  				break;
  			}
  			kk++;
  		}

  		// Now kk is the index of the face we've picked, so sample randomly on that face
  		X(jj,0) = V(F(kk,0),0) + a*(V(F(kk,1),0) - V(F(kk,0),0)) + b*(V(F(kk,2),0) - V(F(kk,0),0));
  		X(jj,1) = V(F(kk,0),1) + a*(V(F(kk,1),1) - V(F(kk,0),1)) + b*(V(F(kk,2),1) - V(F(kk,0),1));
  		X(jj,2) = V(F(kk,0),2) + a*(V(F(kk,1),2) - V(F(kk,0),2)) + b*(V(F(kk,2),2) - V(F(kk,0),2));

	  	// std::cout << X(jj,0) << ", " << X(jj,1) << ", " << X(jj,2) << std::endl;

	}

	// Eigen::RowVector3d x; x(0) = 0.0; x(1) = 0.0; x(2) = 0.0;
	// Eigen::RowVector3d a; a(0) = 1.0; a(1) = 0.0; a(2) = 0.0;
	// Eigen::RowVector3d b; b(0) = 0.0; b(1) = 1.0; b(2) = 0.0;
	// Eigen::RowVector3d c; c(0) = 0.0; c(1) = 0.0; c(2) = 1.0;
	// double d;
	// Eigen::RowVector3d p;
	// point_triangle_distance(x, a, b, c, d, p);

	return;

}

