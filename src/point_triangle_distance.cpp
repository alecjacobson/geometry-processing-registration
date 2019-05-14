#include "point_triangle_distance.h"

//Clamps the values to interval [a,b]
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

//Compute the closest point of X on 2 connected line segments
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
    //I realized this was not as simple as just projecting the points onto the plane defined by the triangle, will update the algorithm to incorporate my idea.
    
    //Effectively, we compute 3 lines that cut up the plane. These lines define 7 cases.
    //We can check the case by looking at which side of the line the projected X point lies in
    
    Eigen::RowVector3d e0,e1,e2, e1_norm;
    e0.array() = b.array() - a.array();
    Eigen::RowVector3d v;
    e1.array() = c.array() - a.array();
    
    //We compute a new coordinate system in the plane containing the triangle
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
    
    //Rotation matrix to compute the normal for each line
    Eigen::Matrix2d RotMat;
    RotMat = Eigen::Matrix2d::Zero();
    RotMat(0,1) = -1;
    RotMat(1,0) = 1;
    
    Eigen::RowVector2d lineEq0, lineEq1, lineEq2;
    //Eq for line AB
    lineEq0(0) = 0;
    lineEq0(1) = -1;
    
    //Eq for line AC
    lineEq1 = RotMat * (c_new - a_new).transpose();
    //Eq for line CB
    lineEq2 = RotMat * (b_new - c_new).transpose();
    
    //Check the side of the line x_new lies on (negative means on the side the triangle is on)
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
            //Point is closest to edge 2, i.e. BC.
            else {
                projectPoint(x_new, c_new,b_new, finalProj);
            }
        } else {
            //Point is closest to edge 1, i.e. AC.
            if (cf <= 0) {
                projectPoint(x_new, a_new,c_new, finalProj);
            }
            //Point lies in the intersection of 2 half-planes defined by CA and CB.
            else { //Annoying Case
                findProjectedPoint(x_new,c_new,b_new,a_new,finalProj);
            }
        }
    } else {
        if (bf <= 0) {
            //Point is closest to edge 0, i.e. AB.
            if (cf <= 0) {
                projectPoint(x_new, a_new,b_new, finalProj);
            }
            //Point lies in the intersection of 2 half-planes defined by BA and BC.
            else { //Annoying Case, Line
                findProjectedPoint(x_new,b_new,a_new,c_new,finalProj);
            }
            //Point lies in the intersection of 2 half-planes defined by AC and AB.
        } else { //Annoying Case
            if (cf <= 0) {
                findProjectedPoint(x_new,a_new,b_new,c_new,finalProj);
                
            } //We should never end up here, but just in case, we return a generic point
            else {findProjectedPoint(x_new,a_new,b_new,c_new,finalProj);
            }
        }
    }
    p = finalProj(0) * e0 + finalProj(1) * e1 + a;
    d = sqrt(p.dot(p) + x.dot(x) - 2*p.dot(x));
    
}
