#include "point_triangle_distance.h"
#include <Eigen/Dense>
void point_triangle_distance(
	const Eigen::RowVector3d & x,
	const Eigen::RowVector3d & a,
	const Eigen::RowVector3d & b,
	const Eigen::RowVector3d & c,
	double & d,
	Eigen::RowVector3d & p)
{
	//We solve this problem by solving a 4x4 linear system. 
	//By writing the point we are trying to solve in the form
	//alpha*a + beta*b + gamma*c = p, st. alpha+beta+gamma = 1,
	//we can use the method of lagrange multipliers to set up and solve a 
	//small 4x4 linear system. 
	//This is probably not the fastest way to solve this, but it's mathematically nice,
	//and avoids having to deal with a bunch of edge cases.
	//Also nice: if we were reusing a triangle a bunch of times, A^-1 could be stored and the whole thing reduces to a matrix multiply.
	//Not-so-nice: possible numeric issues.

	Eigen::Matrix4d A;
	A.row(0) = Eigen::Vector4d(a.dot(a), a.dot(b), a.dot(c), -1);
	A.row(1) = Eigen::Vector4d(b.dot(a), b.dot(b), b.dot(c), -1);
	A.row(2) = Eigen::Vector4d(c.dot(a), c.dot(b), c.dot(c), -1);
	A.row(3) = Eigen::Vector4d(1, 1, 1, 0);
	Eigen::Vector4d B = Eigen::Vector4d(a.dot(x), b.dot(x), c.dot(x), 1);
	Eigen::Vector4d bary = A.inverse()*B;//I think at 4x4, it's faster to directly compute the inverse than it is to solve.

	p = bary(0)*a + bary(1)*b + bary(2)*c;
	d = (x - p).norm();
}