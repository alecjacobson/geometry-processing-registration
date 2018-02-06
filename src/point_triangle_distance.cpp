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

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  auto tempab = b - a;
  auto tempca = c - a;
  auto normal = tempab.cross(tempca).normalized();
  auto intersect = x + (normal.dot(a) - normal.dot(x)) * normal;
  
  Eigen::Matrix3d A;
  A.col(0) = a.transpose();
  A.col(1) = b.transpose();
  A.col(2) = c.transpose();
  Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(intersect.transpose());
  if (!(coeff(0,0) >= 0 &&  coeff(0,0) <= 1
       && coeff(1,0) >= 0 &&  coeff(1,0) <= 1
       && coeff(2,0) >= 0 &&  coeff(2,0) <= 1)){ 
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


