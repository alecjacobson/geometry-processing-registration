#include "point_triangle_distance.h"

double clipdouble(double a, double amin, double amax) {
	if (a > amax)
		return amax;

	if (a < amin)
		return amin;

	return a;
}

void point_triangle_distance(const Eigen::RowVector3d & x,
		const Eigen::RowVector3d & a, const Eigen::RowVector3d & b,
		const Eigen::RowVector3d & c, double & d, Eigen::RowVector3d & p) {
	// Replace with your code

	// fast
	Eigen::RowVector3d ax = x - a;
	Eigen::RowVector3d bx = x - b;
	Eigen::RowVector3d cx = x - c;

	double dax = ax.norm();
	double dbx = bx.norm();
	double dcx = cx.norm();

	if (dax < dbx) {
		if (dax < dcx) {
			d = dax;
			p = a;
		} else {
			d = dcx;
			p = c;
		}
	} else {
		if (dbx < dcx) {
			d = dbx;
			p = b;
		} else {
			d = dcx;
			p = c;
		}
	}

	return;

	// calculate a point d in the plane of abc
	// s.t xd is minimal
	// the same as solve equation
	Eigen::MatrixXd A(3, 2);
	A.col(0) = (b - a).transpose();
	A.col(1) = (c - a).transpose();

	Eigen::MatrixXd B(3, 1);
	B.col(0) = (x - a).transpose();

	Eigen::MatrixXd ATA = A.transpose() * A;
	Eigen::MatrixXd ATB = A.transpose() * B;
	Eigen::MatrixXd ATAINV;
	ATAINV.resizeLike(ATA);
	ATAINV(0, 0) = ATA(1, 1);
	ATAINV(1, 0) = -ATA(1, 0);
	ATAINV(0, 1) = -ATA(0, 1);
	ATAINV(1, 1) = ATA(0, 0);

	double det = ATA(0, 0) * ATA(1, 1) - ATA(0, 1) * ATA(1, 0);
	if (det != 0) {
		ATAINV /= det;
		Eigen::MatrixXd X = ATAINV * ATB;

		double a1 = X(0, 0);
		double a2 = X(1, 0);
		double a3 = 1 - a1 - a2;

		a1 = clipdouble(a1, 0, 1);
		a2 = clipdouble(a2, 0, 1);
		a3 = clipdouble(a3, 0, 1);

		p = a1 * a + a2 * b + a3 * c;
	} else {

		Eigen::RowVector3d ax = x - a;
		Eigen::RowVector3d bx = x - b;
		Eigen::RowVector3d cx = x - c;

		double dax = ax.norm();
		double dbx = bx.norm();
		double dcx = cx.norm();

		if (dax < dbx) {
			if (dax < dcx) {
				d = dax;
				p = a;
			} else {
				d = dcx;
				p = c;
			}
		} else {
			if (dbx < dcx) {
				d = dbx;
				p = b;
			} else {
				d = dcx;
				p = c;
			}
		}
	}
}

