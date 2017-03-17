#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // using the closest point calculation defined in: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
  // for the missing parts of the paper, referenced the following: https://gist.github.com/joshuashaffer/99d58e4ccbd37ca5d96e
  Eigen::Vector3d D,EZero, EOne, B;
  double aa,bb,cc,dd,ee,ff;
  
  // init all of the variables
  EZero = b - a;
  EOne  = c - a;
  B     = a;
  D     = a - x;
  aa    = EZero.dot(EZero);
  bb    = EZero.dot(EOne);
  cc    = EOne.dot(EOne);
  dd    = EZero.dot(D);
  ee    = EOne.dot(D);
  ff    = D.dot(D);

  double det = aa*cc - bb*bb;
  double s = bb*ee - cc*dd;
  double t = bb*dd - aa*ee;

  int8_t region = -1;
  // now break into different regions:
  if( s + t <= det )
  {
    if( s < 0 )
    {
      if( t < 0 )
	region = 4;
      else
	region = 3;
    }
    else if( t < 0 )
      region = 5;
    else
      region = 0;
  }
  else
  {
    if( s < 0 )
      region = 2;
    else if( t < 0 )
      region = 6;
    else
      region = 1;
  }

  double tmp0, tmp1;
  // now to deal with each case of the regions:
  switch( region ) {
  case 0:
    {
      s *= 1.0/det;
      t *= 1.0/det;
    }
    break;
  case 1:
    {
      double numerator = cc + ee - bb - dd;
      if( numerator <= 0 )
      {
	s = 0;
      }
      else
      {
	double denominator = aa - 2.0*bb + cc;
	if( numerator >= denominator )
	  s = 1;
	else
	  s = numerator/denominator;
      }
      t = 1 - s;
    }
    break;
  case 2:
    {
      tmp0 = bb + dd;
      tmp1 = cc + ee;
      if( tmp1 > tmp0 )
      {
	double numerator = tmp1 - tmp0;
	double denominator = aa - 2.0*bb + cc;
	if( numerator >= denominator )
	  s = 1;
	else
	  s = numerator/denominator;
	t = 1 - s;
      }
      else
      {
	s = 0;
	if( tmp1 <= 0 )
	  t = 1;
	else
	  if( ee >= 0 )
	    t = 0;
	  else
	    t = -ee/cc;     
      }
    }
    break;
  case 3:
    {
      s = 0;
      if( ee >= 0 )
	t = 0;
      else
	if( -ee >= cc )
	  t = 1;
	else
	  t = -ee/cc;
    }
    break;
  case 4:
    {
      if( dd < 0 )
      {
	t = 0;
	if( -dd >= aa )
	  s = 1.0;
	else
	  s = -dd / aa;
      }
      else
      {
	s = 0;
	if( ee >= 0.0 )
	  t = 0.0;
	else if( -ee >= cc )
	  t = 1;
	else
	  t = -ee / cc;
      }
    }
    break;
  case 5:
    {
      t = 0;
      if( d >= 0 )
	s = 0;
      else
	if( -dd >= aa )
	  s = 1;
	else
	  s = -dd / aa;
    }
    break;
  case 6:
    { 
      tmp0 = bb + ee;
      tmp1 = aa + dd;
      if( tmp1 > tmp0 )
      {
	double numerator = tmp1 - tmp0;
	double denominator = aa - 2.0*bb + cc;
	if( numerator >= denominator )
	  t = 1;
	else
	  t = numerator / denominator;
	s = 1 - t;
      }
      else
      {
	t = 0;
	if( tmp1 <= 0 )
	  s = 1;
	else
	  if( dd >= 0 )
	    s = 0;
	  else
	    s = -dd/aa;
      }
    }
    break;
  }

  // now reconstruct the point from the barycentric coordinates: p = B + sE₀ + tE₁
  p = B + s*EZero + t*EOne;
  // distance from the point and the triangle:
  Eigen::Vector3d tmp = x - p;
  d = tmp.norm();
}
