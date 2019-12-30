#include "point_triangle_distance.h"
#include <Eigen/Dense>
//find closet point from x to line a,b, out put is distance and p
void closet_to_line(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & x, double & d, Eigen::RowVector3d & p){
    auto tempxa = x - a;
    auto tempxb = x - b;
    auto side_vec = tempxa.cross(tempxb);
    if ((side_vec.norm() < 1e-10) && (side_vec.norm() > -1e-10)){
        d = 0;
        p = x;
        return;
    }
    auto tempab = a - b;
    auto normal = tempab.cross(side_vec).normalized();
    auto distance = (x - a).dot(normal);
    auto xnew = x + distance * (- normal);
    double abnorm = (a-b).norm();
    double anorm =  (xnew - a).norm();
    double bnorm = (xnew - b).norm();
    if (anorm> abnorm){
        p = b;
        d = (x - b).norm();
    } else if (bnorm > abnorm){
        p = a;
        d = (x - a).norm();
    } else {
        p = xnew;
        d = distance;
    }
}

//https://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
bool point_in_triangle(const Eigen::RowVector3d & p, const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c){
    auto u = b - a;
    auto v = c - a;
    auto n = u.cross(v);
    auto w = p - a;
    double a0 = u.cross(w).dot(n)/n.dot(n);
    double a1 = w.cross(v).dot(n)/n.dot(n);
    double a2 = 1 - a0 - a1;

    return (a0 >= 0 && a0 <=1) && (a1 >= 0 && a1 <=1)  &&(a2 >= 0 && a2 <=1);
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
    // #warning "speed by doing nothing"
    // p = (a+b+c)/3;
    // d = (x - p).norm();
    // return;
  auto tempab = b - a;
  auto tempca = c - a;
  auto normal = tempab.cross(tempca).normalized();
  auto intersect = x + (normal.dot(a) - normal.dot(x)) * normal;
  
  if (!point_in_triangle(intersect, a,b,c)){ 
    double line_d[3];
    Eigen::RowVector3d line_p[3]; 
    closet_to_line(a, b, x, line_d[0], line_p[0]);
    closet_to_line(a, c, x, line_d[1], line_p[1]);
    closet_to_line(b, c, x, line_d[2], line_p[2]);

    double min = line_d[0];
    int min_idx = 0;
    for (int i = 0; i<3; i++){
        if (line_d[i] < min){
            min = line_d[i];
            min_idx = i;
        }
    }
    d = min;
    p = line_p[min_idx];

  } else {
    d = (x - intersect).norm();
    p =  intersect;
  }
  
  
}


