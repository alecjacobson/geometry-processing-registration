#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
    Eigen::RowVector3d u = a-c;
    Eigen::RowVector3d v = b-c;
    Eigen::RowVector3d x_0 = x-c;
    Eigen::RowVector3d n = u.transpose().cross(v.transpose()).transpose();
    n.normalize();
    double distance = n.dot(x_0);
    Eigen::RowVector3d poi = x_0 - distance * n;

    double denominator = ((u.dot(v))*(u.dot(v)) - (u.dot(u))*(v.dot(v)));
    double alpha = ((u.dot(v))*(poi.dot(v)) - (v.dot(v))*(poi.dot(u)))/denominator;
    double beta = ((u.dot(v))*(poi.dot(u)) - (u.dot(u))*(poi.dot(v)))/denominator;
    if ( alpha >= 0 && beta >= 0 && alpha + beta <= 1){
        p = c + alpha*u + beta*v;
    } else if (alpha + beta > 1){
        if(alpha < 1){
            p = a;
        } else if (beta < 1){
            p = b;
        } else {
            //project to line ab
            p = c + alpha*u/(alpha+beta) + beta*v/(alpha+beta);
        }
    } else if (beta < 1){
        if (alpha < 1){
            p = c;
        } else {
            //project to line ca
            //get normal of line of ca on the plane:
            Eigen::RowVector3d u = a-b;
            Eigen::RowVector3d v = c-b;
            double denominator = ((u.dot(v))*(u.dot(v)) - (u.dot(u))*(v.dot(v)));
            double alpha = ((u.dot(v))*(poi.dot(v)) - (v.dot(v))*(poi.dot(u)))/denominator;
            double beta = ((u.dot(v))*(poi.dot(u)) - (u.dot(u))*(poi.dot(v)))/denominator;
            p = b + alpha*u/(alpha+beta) + beta*v/(alpha+beta);
        }
    } else {
        //project to line cb
        //get normal of line of ca on the plane:
        Eigen::RowVector3d u = b-a;
        Eigen::RowVector3d v = c-a;
        double denominator = ((u.dot(v))*(u.dot(v)) - (u.dot(u))*(v.dot(v)));
        double alpha = ((u.dot(v))*(poi.dot(v)) - (v.dot(v))*(poi.dot(u)))/denominator;
        double beta = ((u.dot(v))*(poi.dot(u)) - (u.dot(u))*(poi.dot(v)))/denominator;
        p = a + alpha*u/(alpha+beta) + beta*v/(alpha+beta);
    }

    d = (p-x).norm();
}
