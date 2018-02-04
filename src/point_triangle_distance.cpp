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

//Project onto line from a_new to b_new
void projectPoint (const Eigen::RowVector2d & x_new,
                   const Eigen::RowVector2d & a_new,
                   const Eigen::RowVector2d & b_new,
                   Eigen::RowVector2d & proj) {
    float projectedPoint;
    projectedPoint = x_new.dot(b_new - a_new) - a_new.dot(b_new - a_new);
    projectedPoint /=(b_new - a_new).norm();
    projectedPoint = clamp(projectedPoint,0,1);
    proj = a_new + projectedPoint * (b_new - a_new);
    
}

void findProjectedPoint (const Eigen::RowVector2d & x_new,
                         const Eigen::RowVector2d & a_new,
                         const Eigen::RowVector2d & b_new,
                         const Eigen::RowVector2d & c_new,
                         Eigen::RowVector2d & p) {
    
    float projectedPoint, dist1, dist2;
    Eigen::RowVector2d proj1, proj2;
    //Project onto line 1
    projectPoint(x_new, a_new,c_new, proj1);
    //Project onto line 2
    projectPoint(x_new, c_new,b_new, proj2);
    
    dist1 = (x_new - proj1).norm();
    dist2 = (x_new - proj2).norm();
    if (dist1 < dist2) {
        p = proj1;
        
    } else {
        p = proj2;
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
    //Used this to compute closest point to triangle https://www.gamedev.net/forums/topic/552906-closest-point-on-triangle
    //I realized this was not as simple as just projecting the points onto the plane defined by the triangle, will update the algorithm to incorporate my idea.
    
    //Effectively, you have 3 lines that divide the plane. There lines define 7 cases.
    
    Eigen::RowVector3d e0,e1,e2, e1_norm;
    e0.array() = b.array() - a.array();
    Eigen::RowVector3d v;
    e1.array() = c.array() - a.array();
    
    
    e0 = e0 / e0.norm();
    e1 = e1 / e1.norm();
    e1_norm = e1 - e0.dot(e1) * e0;
    
    //Compute the new coords
    Eigen::RowVector2d x_new, a_new,b_new,c_new, proj1, proj2, finalProj;
    x_new(0) = e0.dot(x - a);
    x_new(1) = e1_norm.dot(x-a);
    a_new = Eigen::RowVector2d::Zero();
    b_new(0) = e0.dot(b - a);
    b_new(1) = e1_norm.dot(b-a);
    
    c_new(0) = e0.dot(c - a);
    c_new(1) = e1_norm.dot(c-a);
    
    //
    Eigen::Matrix2d RotMat;
    RotMat = Eigen::Matrix2d::Zero();
    RotMat(0,1) = -1;
    RotMat(1,0) = 1;
    
    Eigen::RowVector2d lineEq0, lineEq1, lineEq2;
    lineEq0(0) = 0;
    lineEq0(1) = -1;
    
    lineEq1 = RotMat * (c_new - a_new).transpose();
    lineEq2 = RotMat * (b_new - c_new).transpose();
    
    float af,bf,cf, projectedPoint, dist1, dist2;
    af = x_new.dot(lineEq0);
    bf = x_new.dot(lineEq1);
    cf = lineEq2.dot(x_new - c_new);
    
    if (af <= 0) {
        if (bf <= 0) {
            //Everything lies on the correct side of the divided plane
            if (cf <= 0) {
                finalProj = x_new;
            }
            else {
                projectPoint(x_new, c_new,b_new, finalProj);
            }
        } else {
            if (cf <= 0) {
                projectPoint(x_new, a_new,c_new, finalProj);
            }
            else { //Annoying Case
                findProjectedPoint(x_new,c_new,b_new,a_new,finalProj);
            }
        }
    } else {
        if (bf <= 0) {
            //Everything lies on the correct side of the divided plane
            if (cf <= 0) {
                projectPoint(x_new, a_new,b_new, finalProj);
            }
            else { //Annoying Case, Line
                findProjectedPoint(x_new,b_new,a_new,c_new,finalProj);
            }
        } else { //Annoying Case
            if (cf <= 0) {
                findProjectedPoint(x_new,a_new,b_new,c_new,finalProj);
                
            }
        }
    }
    p = finalProj(0) * e0 + finalProj(1) * e1 + a;
    d = sqrt(p.dot(p) + x.dot(x) - 2*p.dot(x));
    
}
