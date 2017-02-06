#include "point_triangle_distance.h"

using namespace Eigen;

void point_triangle_distance(
	const Eigen::RowVector3d & x,
	const Eigen::RowVector3d & p1,
	const Eigen::RowVector3d & p2,
	const Eigen::RowVector3d & p3,
	double & distance,
	Eigen::RowVector3d & closestPoint)
{
	// @reference - https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
	// you can also solve it using lagrangians but it will probably be slower. 

	//Using same notation as Eberly
	auto diff = x - p1;
	auto e0 = p2 - p1;
	auto e1 = p3 - p1;
	auto a = e0.dot(e0);
	auto b = e0.dot(e1);
	auto c = e1.dot(e1);
	auto d = -diff.dot(e0);
	auto e = -diff.dot(e1);
	auto det = a * c - b * b;
	auto s = b * e - c * d;
	auto t = b * d - a * e;

	int region;

	if (s + t <= det) 
	{
		if (s < 0) if (t < 0) region = 4; else region = 3;
		else if (t < 0) region = 5;
		else region = 0;		
	}
	else 
	{
		if (s < 0) region = 2;
		else if (t < 0) region = 6;
		else region = 1;
	}

	double tmp0, tmp1, numer, denom;

	switch (region) 
	{
	case 0: 
	{
		double invDet = 1.0 / det;
		s *= invDet;
		t *= invDet; 
		break;
	}
	case 1: 
	{
		numer = c + e - b - d;
		if (numer <= 0.0) { s = 0.0; t = 1.0; }
		else
		{		
			denom = a - ((double)2)*b + c;
			if (numer >= denom) { s = 1.0; t = 0.0; }
			else {s = numer / denom; t = 1.0 - s;}			
		}
		break;
	}
	case 2: 
	{
		tmp0 = b + d;
		tmp1 = c + e;
		if (tmp1 > tmp0)
		{
			numer = tmp1 - tmp0;
			denom = a - ((double)2)*b + c;
			if (numer >= denom) { s = 1.0; t = 0.0; }		
			else { s = numer / denom; t = 1.0 - s; }
		}
		else
		{
			s = 0.0;
			if (tmp1 <= 0.0) t = 1.0;
			else if (e >= 0.0) t = 0.0;
			else t = -e / c;
			
		}
		break;
	}
	case 3: 
	{
		s = 0.0;
		if (e >= 0.0) t = 0.0;
		else if (-e >= c) t = 1.0;
		else t = -e / c;
		break;
	}
	case 4: 
	{
		if (d < 0.0)
		{
			t = 0.0;
			if (-d >= a) s = 1.0;			
			else s = -d / a;			
		}
		else
		{
			s = 0.0;
			if (e >= 0.0) t = 0.0;			
			else if (-e >= c) t = 1.0;
			else t = -e / c;
		}
		break;
	}
	case 5: 
	{
		t = 0.0;
		if (d >= 0.0) s = 0.0;
		else if (-d >= a) s = 1.0;
		else s = -d / a;
		break;
	}
	case 6: {
		tmp0 = b + e;
		tmp1 = a + d;
		if (tmp1 > tmp0)
		{
			numer = tmp1 - tmp0;
			denom = a - ((double)2)*b + c;
			if (numer >= denom) { t = 1.0; s = 0.0; }
			else { t = numer / denom; s = 1.0 - t; }
		}
		else
		{
			t = 0.0;
			if (tmp1 <= 0.0) s = 1.0;
			else if (d >= 0.0) s = 0.0;
			else s = -d / a;
		}
		break;
	}

	}

	closestPoint = p1 + s * e0 + t * e1;
	auto diff2 = x - closestPoint;
	distance = sqrt(diff2.dot(diff2));
}


