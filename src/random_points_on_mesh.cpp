#include "random_points_on_mesh.h"
#include "math.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>

//Runs binary search on matrix V to find randomly sampled triangle based on areas.
//Correct up to index +- 1. To be precise, the higher level structure of binary search is there,
//but the nuance of working out the edge cases might not be.

int binary_search(const Eigen::MatrixXd &V, double randVal)
{
    
    if (V.rows() == 1){
        return 0;
    }
    Eigen::MatrixXd Vhalf;
    
    int n = V.rows();
    
    if (randVal < V(ceil(n/2),0)){
        Vhalf.resize(floor(n/2),1);
        Vhalf = V.topRows(floor(n/2));
        return binary_search(Vhalf, randVal);
    }
    else{
        Vhalf.resize(ceil(n/2),1);
        Vhalf = V.bottomRows(ceil(n/2));
        return floor(n/2) + binary_search(Vhalf, randVal);
        
    }
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{

    
    X.resize(n,3);
    int size = V.rows(), curIndex;
    //These are used to sample the triangle and mesh
    double alpha,beta, randVal, curSum;
    Eigen::MatrixXd A;
    A.resize(size,1);
    curSum = 0;
    //Compute the area of each triangle
    igl::doublearea(V,F,A);
    A.array() = A.array() / 2;
    Eigen::MatrixXd C;
    C.resizeLike(A);
    //compute their cumsum
    igl::cumsum(A,1,C);
    
    //Normalize the sums
    C.array() /= C(C.rows() - 1, 0);
    
    for(int i = 0;i<n;i++) {
        randVal = ((double) rand()/RAND_MAX);
        alpha = (double) rand() / RAND_MAX;
        beta = (double) rand() / RAND_MAX;
        //Randomly sample a triangle by sampling a parallelogram , flip if we are on the wrong side of the parallelogram.
        if (alpha + beta > 1) {
            alpha = 1 - alpha;
            beta = 1 - beta;
        }
        
        //Randomly sample a triangle from the best
        curIndex = binary_search(C, randVal);
        
        
        X.row(i) = (1 - (alpha + beta)) * V.row(F(curIndex,0)) + alpha * V.row(F(curIndex,1)) + beta * V.row(F(curIndex,2));

    }


}



