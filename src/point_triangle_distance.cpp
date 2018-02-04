#include "point_triangle_distance.h"
#include <iostream>
using namespace std;

float clamp (float x, float a, float b) {
    if (x < a) {
        return a;
    }
    else if (x > b) {
        return b;
    } else {
        return x;
    }
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{

  //Create the vectors of the triangle
    //Used http://cs.swan.ac.uk/~csmark/PDFS/1995_3D_distance_point_to_triangle
    
    //Rotate the triangleand tranlate so that a lies at the origin and the edge a-b is along the z-direction and c lies in the y-z plane.
    //Taken from https://www.gamedev.net/forums/topic/552906-closest-point-on-triangle/ verbatim
    //Try someone else's code for now
    Eigen::RowVector3d e0,e1,v0;
    e0.array() = b.array() - a.array();
    e1.array() = c.array() - a.array();
    v0.array() = a.array() - x.array();
    double a1,b1,c1,d1,e_new, det, s, t;
    
    a1 = e0.dot(e0);
    b1 = e0.dot(e1);
    c1 = e1.dot(e1);
    d1 = e0.dot(v0);
    e_new = e1.dot(v0);
    
    det = a1*c1 - b1*b1;
    s = b1*e_new - c1*d1;
    t = b1*d1 - a1*e_new;
    
    if (s + t < det) {
        if (s < 0) {
            if (t < 0) {
                if (d < 0) {
                    s = clamp (-d1/a1, 0, 1);
                    t = 0;
                }
                else {
                    s = 0;
                    t = clamp(-e_new/c1, 0, 1);
                }
            }
            else {
                s = 0;
                t = clamp(-e_new/c1, 0, 1);
            }
        }
        else if (t < 0) {
            s = clamp(-d1/a1, 0, 1);
            t = 0;
        }
        else {
            float invDet = 1.0/det;
            s*= invDet;
            t *= invDet;
            
        }
    }
    else {
        if (s < 0) {
            float tmp0 = b1+d1;
            float tmp1 = c1 + e_new;
            if (tmp1 > tmp0) {
                float numer = tmp1 - tmp0;
                float denom = a1-2*b1 + c1;
                s = clamp(numer/denom, 0, 1);
                t = 1.0-s;
            }
            else {
                t = clamp(-e_new/c1, 0, 1);
                s = 0;
            }
        }
        else if (t < 0) {
            if (a1 + d1 > b1 + e_new) {
                float numer = c1 + e_new - b1 - d1;
                float denom = a1-2*b1 + c1;
                s = clamp(numer/denom, 0, 1);
                t = 1.0-s;
            }
            else {
                s = clamp (-e_new/c1, 0, 1);
                t = 0;
            }
        }
        else {
            float numer = c1+e_new-b1-d1;
            float denom = a1 - 2*b1 + c1;
            s = clamp(numer/denom, 0, 1);
            t = 1.0-s;
        }
    }
    p.array() = a.array() + s*e0.array() + t*e1.array();
    d = sqrt(p.dot(p) + x.dot(x) - 2*p.dot(x));
}
