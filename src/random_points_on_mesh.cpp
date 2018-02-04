#include "random_points_on_mesh.h"
#include "math.h"
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <iostream>
using namespace std;

//Still need to finish this?
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

  // REPLACE WITH YOUR CODE:
    X.resize(n,3);
    int size = V.rows(), curIndex;
    double alpha,beta, randVal, curSum;
    Eigen::MatrixXd A;
    A.resize(size,1);
    curSum = 0;
    igl::doublearea(V,F,A);
    A.array() = A.array() / 2;
    Eigen::MatrixXd C;
    C.resizeLike(A);
    igl::cumsum(A,1,C);
    C.array() /= C(C.rows() - 1, 0);
    
    for(int i = 0;i<n;i++) {
        randVal = ((double) rand()/RAND_MAX);
        alpha = (double) rand() / RAND_MAX;
        beta = (double) rand() / RAND_MAX;
        if (alpha + beta > 1) {
            alpha = 1 - alpha;
            beta = 1 - beta;
        }
        //cout << randVal << "\n";
        curIndex = binary_search(C, randVal);
        
        //cout << "Point: " << i << "RandVal: " << randVal << " Val: " << C(curIndex) << "\n";
        //cout << "Index: " << curIndex << " Value : " << V.row(F(curIndex,0)) << "\n";
        X.row(i) = (1 - (alpha + beta)) * V.row(F(curIndex,0)) + alpha * V.row(F(curIndex,1)) + beta * V.row(F(curIndex,2));
        //Need to find the current triangle
        //cout << "Index: " << curIndex << " Value : " << X.row(i) << "\n";
    }


}



