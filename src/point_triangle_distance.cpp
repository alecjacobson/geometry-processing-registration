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
  // Replace with your code
	Eigen::RowVector3d AB = b - a;
	Eigen::RowVector3d AC = c - a;
	Eigen::RowVector3d AX = x - a;
	Eigen::RowVector3d BX = x - b;
	Eigen::RowVector3d BC = c - b;
	Eigen::RowVector3d CX = x - c;

	double s = AX.dot(AB) / AB.dot(AB);
	double t = AX.dot(AC) / AC.dot(AC);

	if (t > 0 and s > 0 and t+s < 1){
		p = a + (s*AB) + (t*AC);
	}
	else {
		Eigen::RowVector3d v4 = BC * (BX.dot(BC)/(BC.norm())) + b;
		Eigen::RowVector3d v5 = AB * (AX.dot(AB)/(AB.norm())) + a;
		Eigen::RowVector3d v6 = AC * (AX.dot(AC)/(AC.norm())) + a;

		double dis[] = {AX.norm(), BX.norm(), CX.norm(), (v4-x).norm(), (v5-x).norm(), (v6-x).norm()};

		const int N = sizeof(dis) / sizeof(double);

		int min_index = std::distance(dis, std::min_element(dis, dis + N));

		if (min_index == 0){
			p = a;
		}
		else if (min_index == 1){
			p = b;
		}
		else if (min_index == 2){
			p = c;
		}
		else if (min_index == 3){
			p = v4;
		}
		else if (min_index == 4){
			p = v5;
		}
		else {
			p = v6;
		}
	}

  d = (p-x).norm();

}
