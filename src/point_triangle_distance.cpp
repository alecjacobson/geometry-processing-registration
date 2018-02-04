#include "point_triangle_distance.h"
#include <iostream>
using namespace std;

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{

  //Create the vectors of the triangle
    Eigen::RowVector3d u;
    u.array() = a.array() - b.array();
    Eigen::RowVector3d v;
    v.array() = c.array() - b.array();
    
    u = u / u.norm();
    v = v / v.norm();
    v = v - u.dot(v) * u;
    
    p = u.dot(x) * u + v.dot(x) * v;
    
    cout << "Vector P: " << p << "\n";
    cout << "Vector A: " << a << "\n";
    cout << "Vector B: " << b << "\n";
    cout << "Vector C: " << c << "\n";
    
    
    d = (double) sqrt(p.dot(p) + x.dot(x) - 2* x.dot(p));
    
}
