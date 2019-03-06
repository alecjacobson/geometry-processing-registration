#include "random_points_on_mesh.h"
#include <cstdlib> // rand
#include <igl/cumsum.h> // cumsum
#include <igl/doublearea.h> // doublearea

// uniform random sample within a given triangle
Eigen::Vector3d random_points_in_triangle(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
    
    // generate two random numbers in the interval [0,1]
    double alpha = ((double) rand()) / RAND_MAX;
    double beta = ((double) rand()) / RAND_MAX;

    // to avoid returning a point in the reflected triangle used to construct the parallelogram
    // over which we're sampling, use 1-alpha and 1-beta if alpha + beta exceeds 1.
    Eigen::Vector3d x = (alpha + beta <= 1) ? v1 + alpha*(v2 - v1) + beta*(v3 - v1) : 
                             v1 + (1-alpha)*(v2 - v1) + (1-beta)*(v3 - v1);
    return x;
}

// area-weighted sample of the triangles
int random_triangle_index(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F) {
    
    // generate continuous random number in the interval [0,1]
    double gamma = ((double) rand()) / RAND_MAX;
    
    // compute the area of each triangle in the mesh
    Eigen::VectorXd A; 
    igl::doublearea(V,F,A);
    A *= 0.5;
    
    // compute the cumulative sum of the triangle areas (0.5*A is used
    // since doublearea() returns doubled areas. Strictly speaking this wasn't necessary
    // since we're concerned with the triangle area as a fraction of the total.
    // For clarity I chose to deal with 0.5*A anyway).
    Eigen::VectorXd C;
    igl::cumsum(A, 1, C);
    
    // use gamma to generate discrete random index
    for (int i = 0; i < C.size(); i++) {
        if (C(i) > gamma) return i;
    }
    
    // we'll never reach this point, but the return is included to suppress a warning
    // from the compiler.
    return -1;   
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X) {
    
    // set X to the appropriate size
    X.resize(n,3);
    
    for (int i = 0; i < n; i++) {
        // generate random triangle index
        int index = random_triangle_index(V, F);
        
        // generate a random point in the selected triangle
        Eigen::Vector3d v1 =  V.row(F(index,0));
        Eigen::Vector3d v2 =  V.row(F(index,1));
        Eigen::Vector3d v3 =  V.row(F(index,2));
    
        X.row(i) = random_points_in_triangle(v1, v2, v3);
    }
}

