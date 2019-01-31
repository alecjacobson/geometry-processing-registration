#include "point_triangle_distance.h"
#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // // Replace with your code
  // d = 0;
  // p = a;

	// Pick number of samples on the given triangle
	int n = 100;

	// Initialize the output variables
	p.resize(3);
	d = 1.0e10;

	for (int ii = 0; ii < n; ii++)
	{
		// Define the random constants to sample the triangle
		double a1 = -((double)rand()/(double)(RAND_MAX + 1));
		double b1 = -((double)rand()/(double)(RAND_MAX + 1));

	  	if (a1 + b1 > 1.0)
		{
			a1 = 1.0 - a1;
			b1 = 1.0 - b1;
		}

	  	double xp = a(0) + a1*(b(0) - a(0)) + b1*(c(0) - a(0));
	  	double yp = a(1) + a1*(b(1) - a(1)) + b1*(c(1) - a(1));
	  	double zp = a(2) + a1*(b(2) - a(2)) + b1*(c(2) - a(2));

	  	double x_dist = std::abs(xp - x[0]);
	  	double y_dist = std::abs(yp - x[1]);
	  	double z_dist = std::abs(zp - x[2]);
	  	double dist = std::sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);

	// std::cout << dist << "; " << xp << ", " << yp << ", " << zp << std::endl;

	  	if (dist < d)
	  	{
	  		p[0] = xp; p[1] = yp; p[2] = zp;
	  		d = dist;
	  	}
	}

	// std::cout << d << "; " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;

	return;

}


