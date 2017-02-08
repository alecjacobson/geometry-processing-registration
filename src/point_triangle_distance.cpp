#include "point_triangle_distance.h"

#include <iostream>
#include <cmath>
#include <assert.h>

//
// From Schneider & Eberly...
//
void ptd(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & v0,
  const Eigen::RowVector3d & v1,
  const Eigen::RowVector3d & v2,
  double & dd,
  Eigen::RowVector3d & p,
  bool debug = false )
{
    Eigen::RowVector3d B = v0;
    Eigen::RowVector3d E0 = v1 - v0;
    Eigen::RowVector3d E1 = v2 - v0;
    Eigen::RowVector3d P = x;

    Eigen::RowVector3d D = B - P;
    double a = E0.dot( E0 );
    double b = E0.dot( E1 );
    double c = E1.dot( E1 );
    double d = E0.dot( D );
    double e = E1.dot( D ); // getting rid of the -1 is better? 2 differences in text!
    double f = D.dot( D );

    assert( ( E0.cross( E1 ) ).norm() > 0 ); // degenerate triangle?
    //double delta = std::abs(a*c - b*b);

    int region( 0 ); // 0-6

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e; // should this be b*d - a*d ?? no, mistake in eberly.

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
        numer = c + e - b - d; // mistake in eberly code example mixes up e w/d
        if( debug )
            std::cout << "numer= "<<numer<<std::endl;
        if( numer <= 0 )
            s = 0;
        else
        {
            denom = a - 2*b + c;
            if( debug )
                std::cout << "denom= "<< denom<<std::endl;
            s = ( (numer >= denom) ? 1 : (numer/denom) );
        }
        t = 1 - s;
        break;
    case 4: // close to 2 but not identical
    {
        double tmp0( b+d ), tmp1( c+e );
        if( debug )
            std::cout<<"tmp0/1: "<<tmp0<<", "<<tmp1<<std::endl;
        if( tmp1 < tmp0 )
        {
            if( debug )
                std::cout << "first case" << std::endl;
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
    case 2:
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
    case 3:
        s = 0;
        t = ( e >= 0 ? 0 : (-e >= c ? 1 : -e/c ) );
        break;
   case 5:
       {
           //std::cout << "5: d is " << d << " and a is " << a << std::endl;
           //std::cout << "5: -d/a= "<<(-d/a)<<std::endl;
           t = 0;
           s = ( d >= 0 ? 0 : (-d >= a ? 1 : -d/a ) );
           //std::cout <<"5: s,t= "<<s<<", "<<t<<std::endl;
           //t = 0;     // TODO check
           //s = ( e >= 0 ? (e >= c ? e/c : 1 ) : 0 );
           //s = ( e >= 0 ? (e >= c ? -e/c : 1) : 0 );
       }
       break;
    }

    if( s > 1 )
        s = 1;
    if( t > 1 )
        t = 1;
    if( s < 0 )
        s = 0;
    if( t < 0 )
        t = 0;

    if( debug )
        std::cout << "Point Triangle Region: " << region << std::endl;
    
    // Closest point is then given as
    p = B + s*E0 + t*E1;
    dd = (p-x).norm();
    if( debug )
    {
        std::cout << "s,t = " << s << ", " << t << " d= " << dd << std::endl;
        std::cout << "p= " << p << std::endl;
    }

    // TODO -- REMOVE -- TESTING!
    // for testing, let's always return v0 as the nearest point
    //p = v0;
    //d = (p-x).norm();
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
  Eigen::RowVector3d & p,
  bool debug )
{
    // Replace with your code
    //d = 0;
    //p = a;
    // std::cout << "Given x= " << x << std::endl;
    // std::cout << "a: " << a << std::endl;
    // std::cout << "b: " << b << std::endl;
    // std::cout << "c: " << c << std::endl;

    ptd( x, a, b, c, d, p, debug );
}


// mostly tried to implement:
// Schneider, Philip, and David H. Eberly. Geometric tools for computer graphics. Morgan Kaufmann, 2002.
//
// Other refs....
// Jones, Mark W. "3D distance from a point to a triangle." Department of Computer Science, University of Wales Swansea Technical Report CSR-5 (1995).
// David Eberly's: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf




    // // determine the region on the plane of the triangle
    // if( s+t <= det )
    // {
    //     if( s < 0 )
    //     {
    //         if( t < 0 )
    //             region = 4;
    //         else
    //             region = 3;
    //     }
    //     else if( t < 0 )
    //         region = 5;
    //     else
    //         region = 0;
    // }
    // else
    // {
    //     if( s < 0 )
    //         region = 2;
    //     else if( t < 0 )
    //         region = 6;
    //     else
    //         region = 1;
    // }

    // double denom, numer;
    // switch( region )
    // {
    // case 0 :
    // {
    //     double invDet = 1/det;
    //     s *= invDet;
    //     t *= invDet;
    // }
    //     break;
    // case 1 :
    //     numer = c + d - b - d;
    //     if( numer <= 0 )
    //         s = 0;
    //     else
    //     {
    //         denom = a - 2*b + c;
    //         s = ( numer >= denom ? 1 : numer/denom );
    //     }
    //     t = 1 - s;        
    //     break;
    // case 2: // regions 4 & 6 similar
    // case 4: // but not identical TODO
    // case 6:
    // {
    //     double tmp0( b+d ), tmp1( c+e );
    //     if( tmp1 > tmp0 )
    //     {
    //         numer = tmp1 - tmp0;
    //         denom = a - 2*b + c;
    //         s = ( numer >= denom ) ? 1 : numer/denom;
    //         t = 1 - s;
    //     }
    //     else
    //     {
    //         s = 0; // min on edge
    //         t = ( tmp1 <= 0 ) ? 1 : ( e >= 0 ? 0 : -e/c );
    //     }
    // }
    //     break;
    // case 3 :
    //     s = 0;
    //     t = ( e >= 0 ? 0 : (-e >= c ? 1 : -e/c ) );
    //     break;
    // case 5:
    //     t = 0;     // TODO check
    //     s = ( e >= 0 ? 0 : (-e >= c ? 1 : -e/c ) );
    //     break;
    // }

    // std::cout << "Point Triangle Region: " << region << std::endl;
    
    // // Closest point is then given as
    // p = B + s*E0 + t*E1;
    // //std::cout << "s,t = " << s << ", " << t << std::endl;
    // d = (p-x).norm();


    // // TODO -- REMOVE -- TESTING!
    // // for testing, let's always return v0 as the nearest point
    // //p = v0;
    // //d = (p-x).norm();
