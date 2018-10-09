#include "point_triangle_distance.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>

void point_triangle_distance(
                             const Eigen::RowVector3d & x,
                             const Eigen::RowVector3d & a,
                             const Eigen::RowVector3d & b,
                             const Eigen::RowVector3d & c,
                             double & d,
                             Eigen::RowVector3d & p)
{
    Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(a, b, c);
    Eigen::RowVector3d proj = plane.projection(x);
    if (((b-a).cross(c-a)).dot((b-a).cross(x-a)) >= 0 && ((a-c).cross(b-c)).dot((a-c).cross(x-c)) >= 0 && ((c-b).cross(a-b)).dot((c-b).cross(x-b)) >= 0) {
        d = (x - proj).norm();
        p = proj;
    }
    else {
        d = 100000000;
        double z = (x - a).dot(b - a) / (b - a).squaredNorm();
        if (z < 0) z = 0;
        else if (z > 1) z = 1;
        Eigen::RowVector3d projection = a + z * (b - a);
        Eigen::RowVector3d abP = projection;
        double abD = (x - projection).norm();
        if (abD < d) {
            d = abD;
            p = abP;
        }
        z = (x - b).dot(c - b) / (c - b).squaredNorm();
        if (z < 0) z = 0;
        else if (z > 1) z = 1;
        projection = b + z * (c - b);
        Eigen::RowVector3d bcP = projection;
        double bcD = (x - projection).norm();
        if (bcD < d) {
            d = bcD;
            p = bcP;
        }
        z = (x - c).dot(a - c) / (a - c).squaredNorm();
        if (z < 0) z = 0;
        else if (z > 1) z = 1;
        projection = c + z * (a - c);
        Eigen::RowVector3d caP = projection;
        double caD = (x - projection).norm();
        if (caD < d) {
            d = caD;
            p = caP;
        }
    }
    
}
