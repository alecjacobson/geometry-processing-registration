#include "point_triangle_distance.h"

#include <iostream>
#include <cmath>

#include <assert.h>

//
// From Schneider & Eberly...
//
void ptd(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & aa,
  const Eigen::RowVector3d & bb,
  const Eigen::RowVector3d & cc,
  double & dd,
  Eigen::RowVector3d & p)
{
    Eigen::RowVector3d B = aa;
    Eigen::RowVector3d E0 = bb - cc;
    Eigen::RowVector3d E1 = cc - aa;
    Eigen::RowVector3d P = x;

    Eigen::RowVector3d D = B - P;
    double a = E0.dot( E0 );
    double b = E0.dot( E1 );
    double c = E1.dot( E1 );
    double d = E0.dot( D );
    double e = E1.dot( D );
    double f = D.dot( D );

    double delta = std::abs(a*c - b*b);

    int region( 0 ); // 0-6

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    assert( (E0.cross(E1)).norm() );

    //double invDet, numer
  
    // determine the region on the plane of the triangle
    if( s+t <= det )
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

    double denom, numer;

    // double zz;
    // switch( region )
    // {
    // case 0:
    //     double foo = 123;
    //     zz = 1;
    //     break;
    // case 2:
    // case 4:
    // case 6:
    //     zz = 3;
    //     break;
    // case 3:
    //     zz = 7;
    //     break;
    // }

    switch( region )
    {
    case 0 :
    {
        double invDet = 1/det;
        s *= invDet;
        t *= invDet;
    }
        break;
    case 1 :
        numer = c + d - b - d;
        if( numer <= 0 )
            s = 0;
        else
        {
            denom = a - 2*b + c;
            s = ( numer >= denom ? 1 : numer/denom );
        }
        t = 1 - s;        
        break;
    case 2: // regions 4 & 6 similar
    case 4: // but not identical TODO
    case 6:
    {
        double tmp0( b+d ), tmp1( c+e );
        if( tmp1 > tmp0 )
        {
            numer = tmp1 - tmp0;
            denom = a - 2*b + c;
            s = ( numer >= denom ) ? 1 : numer/denom;
            t = 1 - s;
        }
        else
        {
            s = 0; // min on edge
            t = ( tmp1 <= 0 ) ? 1 : ( e >= 0 ? 0 : -e/c );
        }
    }
        break;
    case 3 :
        s = 0;
        t = ( e >= 0 ? 0 : (-e >= c ? 1 : -e/c ) );
        break;
    case 5:
        t = 0;     // TODO check
        s = ( e >= 0 ? 0 : (-e >= c ? 1 : -e/c ) );
        break;
    }

    // Closest point is then given as
    p = B + s*E0 + t*E1;
    //std::cout << "s,t = " << s << ", " << t << std::endl;
    d = (p-x).norm();
}

//
// Compute the distance d between a given point x and the closest point p on
// a given triangle with corners a, b, and c.
//
void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
    // Replace with your code
    //d = 0;
    //p = a;
    // std::cout << "Given x= " << x << std::endl;
    // std::cout << "a: " << a << std::endl;
    // std::cout << "b: " << b << std::endl;
    // std::cout << "c: " << c << std::endl;

    ptd( x, a, b, c, d, p );
}


// mostly tried to implement:
// Schneider, Philip, and David H. Eberly. Geometric tools for computer graphics. Morgan Kaufmann, 2002.
//
// Other refs....
// Jones, Mark W. "3D distance from a point to a triangle." Department of Computer Science, University of Wales Swansea Technical Report CSR-5 (1995).
// David Eberly's: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
