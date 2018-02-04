#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <iostream>
using namespace std;

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
    
  P.resizeLike(X);
    Eigen::MatrixXd allN;
    allN.resizeLike(FY);
    igl::per_face_normals(VY,FY,allN);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    D.resize(X.rows());
    
    double bestDist, *curDist;
    curDist = (double *) malloc(sizeof(double));
    
    for(int i = 0;i<X.rows();i++) {
        int bestIndex = 0;
        
        Eigen::RowVector3d bestP, curP;
        Eigen::RowVector3d bestN;
        
        point_triangle_distance(X.row(i), VY.row(FY(0,0)), VY.row(FY(0,1)),VY.row(FY(0,2)), *curDist, bestP);
        bestDist = *curDist;
        for (int j = 1; j < FY.rows(); j++){
            point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)),VY.row(FY(j,2)), *curDist, curP);
            if (*curDist < bestDist) {
                bestIndex = j;
                bestDist = *curDist;
                bestP = curP;
            }
        }
        D(i) = bestDist;
        P.row(i) = bestP;
        N.row(i) = allN.row(bestIndex);
        
    }
    delete curDist;
    
}
