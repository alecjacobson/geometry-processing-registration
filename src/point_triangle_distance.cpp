#include "point_triangle_distance.h"
#include "util.h"

using namespace Eigen;

struct Plane {
	Eigen::Vector3d normal;
	double d;

	bool inFront(Eigen::Vector3d point) {
		return normal.dot(point) + d > 0;
	}

	double getPointD(Eigen::Vector3d point) {
		return normal.dot(point) + d;
	}

	Eigen::Vector3d project(Eigen::Vector3d point) {
		double pointD = normal.dot(point);
		return point - normal * (d + pointD);
	}
};

Plane makeTrianglePlane(Eigen::Vector3d a, Eigen::Vector3d b, 
		Eigen::Vector3d c) {
	Eigen::Vector3d u = b - a;
	Eigen::Vector3d v = c - a;

	Plane p;
	p.normal = cross(u, v);
	p.normal.normalize();

	assert(abs(p.normal.dot(u)) < 0.0001 && abs(p.normal.dot(v)) < 0.0001);

	p.d = -a.dot(p.normal);

	return p;
}

Plane makeEdgePlane(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, 
		Eigen::Vector3d backPoint) {
	Plane trianglePlane = makeTrianglePlane(a, b, c);

	if (trianglePlane.inFront(backPoint)) {
		trianglePlane.normal = -trianglePlane.normal;
		trianglePlane.d = -trianglePlane.d;
	}

	return trianglePlane;
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p) {

	Eigen::RowVector3d normal = cross((b - a), (c - a));
	normal.normalize();

	Plane ab = makeEdgePlane(a, b, a + normal, c);
	Plane bc = makeEdgePlane(b, c, b + normal, a);
	Plane ca = makeEdgePlane(c, a, c + normal, b);

	// Check if the point is in the positive half space of each edge plane.
	bool frontAB = ab.inFront(x);
	bool frontBC = bc.inFront(x);
	bool frontCA = ca.inFront(x);

	// Also check if the point projects onto a valid portion of each edge.
	// We classify the point as being either before the first point of the 
	// edge (-1), between the endpoints (0), or after the second point on the
	// edge (1).
	Vector3d aToB = b - a;
	Vector3d bToC = c - b;
	Vector3d cToA = a - c;
	assert(a.dot(aToB) < b.dot(aToB));
	assert(b.dot(bToC) < c.dot(bToC));
	assert(c.dot(cToA) < a.dot(cToA));
	int withinAB = -(a.dot(aToB) > x.dot(aToB)) + (x.dot(aToB) > b.dot(aToB));
	int withinBC = -(b.dot(bToC) > x.dot(bToC)) + (x.dot(bToC) > c.dot(bToC));
	int withinCA = -(c.dot(cToA) > x.dot(cToA)) + (x.dot(cToA) > a.dot(cToA));

	// Compute projection onto the triangle plane.
	Plane trianglePlane = makeTrianglePlane(a, b, c);
	Eigen::RowVector3d projection = trianglePlane.project(x);
	assert(abs(trianglePlane.getPointD(a)) < 0.001);
	assert(abs(trianglePlane.getPointD(b)) < 0.001);
	assert(abs(trianglePlane.getPointD(c)) < 0.001);
	assert(abs(ab.normal.dot(trianglePlane.normal)) < 0.001);

	/* 
	* 1. If its behind all edge planes, the closest projection lies in the 
	* triangle.
	*
	* 2. If its in front of an edge plane and within the bounds of that edge,
	* then the closest projection lies on that edge.
	*
	* 3. Otherwise, it must be at one of the corners. which one depends on
	* which side of the edges it is on.
	*/
	if (!frontAB && !frontBC && !frontCA) {

		p = projection;

	} else if (frontAB && withinAB == 0) {
		p = ab.project(projection);
	} else if (frontBC && withinBC == 0) {
		p = bc.project(projection);
	} else if (frontCA && withinCA == 0) {
		p = ca.project(projection);
	} else if (withinAB < 0 && withinCA > 0) {
		p = a;
	} else if (withinBC < 0 && withinAB > 0) {
		p = b;
	} else if (withinCA < 0 && withinBC > 0) {
		p = c;
	} else {

		// Shouldn't happen.
		assert(0 == 1);
	}
	
	d = (x - p).norm();

	assert(d - 0.0001 <= (x - a).norm());
	assert(d - 0.0001 <= (x - b).norm());
	assert(d - 0.0001 <= (x - c).norm());
}
