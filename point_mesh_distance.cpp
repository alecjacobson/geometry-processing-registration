#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <cfloat> // DBL_MAX
#include <Eigen/Geometry> // cross

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N) {
    
    P.resizeLike(X);
    D.resize(X.rows());
    N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    
    for(int i = 0; i < X.rows(); i++) {
        
        // initialize the minimum variables
        double min_d = DBL_MAX;
        Eigen::RowVector3d min_p;
        Eigen::RowVector3d min_n;
        
        // cycle through the faces and compute the closest point distance for each.
        // If that distance is less than the current minimum, update the minimum variables
        for (int j = 0; j < FY.rows(); j++) {
            
            // retrieve the coordinates for the vertices of the current face
            Eigen::RowVector3d a = VY.row(FY(j,0));
            Eigen::RowVector3d b = VY.row(FY(j,1));
            Eigen::RowVector3d c = VY.row(FY(j,2));
            
            // compute the closest point distance between X(i) and the current face
            double d;
            Eigen::RowVector3d p;
            point_triangle_distance(X.row(i), a, b, c, d, p);

            // update if that distance is less than the current minimum
            if (d < min_d) {
                
                // minimum distance and corresponding point
                min_d = d;
                min_p = p;
                
                // compute the normal for the current face
                min_n = (b-a).cross(c-a).normalized();
            }
        }
        
        // set the minimum values for the current point
        P.row(i) = min_p;
        N.row(i) = min_n; 
        D(i) = min_d;
    }
}
