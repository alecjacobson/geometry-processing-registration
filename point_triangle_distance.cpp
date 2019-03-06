#include "point_triangle_distance.h"
#include <Eigen/Dense> // norm

// source: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p) {

    // vectors defining the triangle
    Eigen::RowVector3d e0 = b - a;
    Eigen::RowVector3d e1 = c - a;
    
    // the squared distance between x and any point on the triangle gives a quadratic function
    // in terms of the parameters defining the triangle (s and t). The following are the
    // coefficients in that quadratic formula.
    double coefa = e0.dot(e0);
    double coefb = e0.dot(e1);
    double coefc = e1.dot(e1);
    double coefd = e0.dot(a - x);
    double coefe = e1.dot(a - x);
    double coeff = (a - x).dot(a-x);
    
    // parameter values where the gradient of the quadratic function is 0. The actual
    // parameter values are s/den and t/den, but the division is left till later for
    // efficiency.
    double s = (coefb*coefe - coefc*coefd);
    double t = (coefb*coefd - coefa*coefe);
    double den = (coefa*coefc - coefb*coefb);
    
    // the edges of the triangle can be thought to partition the plane in which the triangle lies
    // into 7 distinct regions (the interior of the triangle, the 3 regions where points are offside
    // a single edge, and the 3 regions where points are offside 2 edges simultaneously). Graphically,
    // think of the latter 3 regions as flat cone-like areas emanating out from each vertex, bounded on 
    // either side by lines following the paths of the two edges of the triangle that cross at the vertex. 
    // The algorithm proceeds by first determining which region the parameter values globally minimizing the 
    // gradient of the distance function fall into. If they produce a point in the interior of the 
    // triangle, the global minimizing s and t values are sufficient. Otherwise, find the smallest value 
    // for a level curve of the distance function with a point on the triangle boundary and select the s 
    // and t for this boundary point. Regions 1, 3, and 5 consist of the points that are offside a 
    // single edge of the triangle, while regions 2, 4, and 6 correspond to the points offside two edges.
    if (s + t <= den) {
        if (s < 0) {
            if (t < 0) {
                // region 4: points offside ab and ac, emanating from vertex a
                if (coefd < 0) {
                    // closest point is on the line ab
                    t = 0;
                    if (-coefd >= coefa) {
                        // closest point is at vertex b
                        s = 1;
                    } else {
                        // somewhere between a and b
                        s = -coefd / coefa;
                    }
                } else {
                    // closest point is on the line ac
                    s = 0;
                    if (coefe >= 0) {
                        // closest point is at vertex a
                        t = 0;
                    } else if (-coefe >= coefc) {
                        // closest point is at vertex c
                        t = 1;
                    } else {
                        // somewhere between a and c
                        t = -coefe / coefc;
                    }
                }
            } else {
                // region 3: points offside the edge ac
                s = 0;
                if (coefe >= 0) {
                    // closest point is vertex a
                    t = 0;
                } else if (-coefe >= coefc) {
                    // closest point is vertex c
                    t = 1;
                } else {
                    // somewhere between a and c
                    t = -coefe / coefc;
                }
            }
        } else if (t < 0) {
            // region 5: points offside the edge ab
            t = 0;
            if (d >= 0) {
                // closest point is vertex a
                s = 0;
            } else if (-coefd >= coefa) {
                // closest point is vertex b
                s = 1;
            } else {
                // somewhere between a and b
                s = -coefd / coefa;
            }
        } else {
            // region 0: s and t are both non-negative and s + t <= 1 so the
            // point where the gradient is minimized is inside the triangle.
            s /= den;
            t /= den;
        }
    } else {
        if (s < 0) {
            // region 2: points offside ac and bc, emanating from vertex c
            double bpd = coefb + coefd;
            double cpe = coefc + coefe;
            
            if (cpe > bpd) {
                double num = cpe - bpd;
                den = coefa - 2*coefb + coefc;
                
                if (num >= den) {
                    // closest point is vertex b
                    s = 1;
                } else {
                    // closest point is somewhere between b and c
                    s = num / den;
                }
                t = 1 - s;
            } else {
                s = 0;
                if (cpe <= 0) {
                    // closest point is vertex c
                    t = 1;
                } else if (coefe >= 0) {
                    // closest point is vertex a
                    t = 0;
                } else {
                    // somewhere between a and c
                    t = -coefe / coefc;
                }
            }
        } else if (t < 0) {
            // region 6: points offside ab and bc, emanating from vertex b
            double bpe = coefb + coefe;
            double apd = coefa + coefd;
            
            if (apd > bpe) {
                double num = apd - bpe;
                den = coefa - 2*coefb + coefc;
                
                if (num >= den) {
                    // closest point is vertex c
                    t = 1;
                } else {
                    // somewhere between b and c
                    t = num / den;
                }
                s = 1 - t;
            } else {
                t = 0;
                if (apd <= 0) {
                    // closest point is vertex b
                    s  = 1;
                } else if (coefd >= 0) {
                    // closest point is vertex a
                    s = 0;
                } else {
                    // somewhere between a and b
                    s = -coefd / coefa;
                }
            }
        } else {
            // region 1: points offside the edge bc
            double num = (coefc + coefe) - (coefb + coefd);
            if (num <= 0) {
                // closest point is vertex c
                s = 0;
            } else {
                den = coefa - 2*coefb + coefc;
                if (num >= den) {
                    // closest point is vertex b
                    s = 1;
                } else {
                    // somewhere between b and c
                    s = num/den;
                }
            }
            t = 1 - s;
        }
    }
    
    // having computed the appropriate values for s and t, we can now compute the
    // closest triangle point and its corresponding distance to x
    p = a + s*e0 + t*e1;
    d = (x-p).norm();
    
}
