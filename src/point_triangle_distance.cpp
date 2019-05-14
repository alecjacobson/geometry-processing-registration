#include "point_triangle_distance.h"
#include "vect_to_skew.h"
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

Eigen::VectorXd cross(Eigen::VectorXd v, Eigen::VectorXd v_xa)
{
    // Inspired by: https://stackoverflow.com/questions/26ac6778/cross-product-matrix-in-eigen
    return vect_to_skew(v) * v_xa;
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
    Eigen::RowVector3d v_ca = c - a;
    Eigen::RowVector3d v_ba = b - a; 
    Eigen::RowVector3d normal_vector = cross(v_ca, v_ba);
    normal_vector.normalize();

    Eigen::RowVector3d v_x = x - a;
    // Project vector from a to x to the plane where the triangle is
    double distance_to_plane = v_x.dot(normal_vector);
    Eigen::RowVector3d plane_x = v_x - normal_vector * distance_to_plane;
    
    // We now have a new vector pointing from a to some point on the plane
    Eigen::RowVector3d v_xa = plane_x - a;

    // Taken from: http://blackpawn.com/texts/pointinpoly/
    // "We can describe any point on the plane as: P = A + u * (C - A) + v * (B - A)"

    // Step 1) Pre-compute dot products (TAKEN FROM ABOVE SOURCE)
    double dot00 = v_ca.dot(v_ca);
    double dot01 = v_ca.dot(v_ba);
    double dot02 = v_ca.dot(v_xa);
    double dot11 = v_ba.dot(v_ba);
    double dot12 = v_ba.dot(v_xa);
    // Step 2) Compute barycentric coordinates (TAKEN FROM ABOVE SOURCE) to obtain u and v
    double denom = (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) / denom;
    double v = (dot00 * dot12 - dot01 * dot02) / denom;

    // If u or v is < 0, we will go the wrong way (away from triangle)
    // If u + v is > 1, we will leave the triangle
    // In either case, the nearest point must lie on the perimeter of the triangle
    // Otherwise, we have already projected into the nearest point
    if (u >= 0 and v >= 0 and (u + v <= 1)){
        p = plane_x;
    }
    // There's 6 possible cases, but these aren't even the correct one
    // Make some okay decisions because I couldn't figure it out    
    else if (u >= 1 and v >= 1){
        p = c;
    }
    else if (u <= 0 and v <= 0){
        p = a;
    }
    else if (u >= 1 and v <= 0){
        p = b;
    }
    else if (u >= 0 and v <= 0){
        p = (a + b) / 2.0;
    }
    else if (u <= 0 and v >= 0.0){
        p = (a + c) / 2.0;
    }
    else if (u >= 0.0 and v >= 0){
        p = (b + c) / 2.0;
    }
    else{
        p = (a + b + c) / 2.0;
        cout << "Something seriously wrong with logic: u = " << u << ", v = " << v << endl; 
    }

    d = (x - p).norm();
}
