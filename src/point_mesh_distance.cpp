#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
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
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    D.resize(X.rows());
    for(int i = 0;i<X.rows();i++) {
        int bestIndex = 0;
        double bestDist, curDist;
        Eigen::RowVector3d bestP, curP;
        Eigen::RowVector3d bestN;
        
        point_triangle_distance(X.row(i), V.row(F(0,0)), V.row(F(0,1)),V.row(F(0,2)), &bestDist, bestP);
        
        for (int j = 1; j < FY.rows(); j++){
            point_triangle_distance(X.row(i), V.row(F(j,0)), V.row(F(j,1)),V.row(F(j,2)), &curDist, curP);
            if (curDist < bestDist) {
                bestIndex = j;
                bestDist = curDist;
                bestP = curP;
            }
        }
        D(i) = bestDist;
        P.row(i) = bestP;
    }
}
