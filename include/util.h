#pragma once
#include <Eigen/Core>

/**
* Had some trouble getting cross-product to compile properly or something,
* so I rolled my own.
*/
inline Eigen::Vector3d cross(Eigen::Vector3d a, Eigen::Vector3d b) {

	double s1 = a(1) * b(2) - a(2) * b(1);
	double s2 = a(2) * b(0) - a(0) * b(2);
	double s3 = a(0) * b(1) - a(1) * b(0);

	Eigen::Vector3d result(s1, s2, s3);
	return result;
}