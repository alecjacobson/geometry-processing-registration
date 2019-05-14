#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    
  P.resizeLike(X);
    
    //Stores the face normals
    Eigen::MatrixXd allN;
    allN.resizeLike(FY);
    
    //Compute face normals
    igl::per_face_normals(VY,FY,allN);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    D.resize(X.rows());
    
    double bestDist, *curDist;
    //Pointer to curDistance
    curDist = (double *) malloc(sizeof(double));
    
    //Loop through every point, finding its closest point
    for(int i = 0;i<X.rows();i++) {
        int bestIndex = 0;
        
        Eigen::RowVector3d bestP, curP;
        Eigen::RowVector3d bestN;
        
        //Compute point to triangle distance
        //Initialize with triangle 1
        point_triangle_distance(X.row(i), VY.row(FY(0,0)), VY.row(FY(0,1)),VY.row(FY(0,2)), *curDist, bestP);
        bestDist = *curDist;
        for (int j = 1; j < FY.rows(); j++){
            point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)),VY.row(FY(j,2)), *curDist, curP);
            //Update best point if we find a closer one.
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
