#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>
#include <iostream>

int binary_search_larger(Eigen::MatrixXd & X, double limit){
    int a,b,c;
    a = 0;
    b = X.rows()-1;
    c = (a+b)/2;
    while(1){
        double ratio = X(c,0)/X(X.rows()-1,0);
        if (ratio> limit){
            a = a;
            b = c;
            c = (a+b)/2;
        } else if (ratio <= limit){
            a = c+1;
            b = b;
            c = (a+b)/2;
        }
        if (a == b){
            break;
        }
    }
    return c;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
    //std::cout << "w" << std::endl;
    X.resize(n, 3);
    Eigen::MatrixXd areas;
    Eigen::MatrixXd colsum;

    igl::doublearea(V, F, areas);
    igl::cumsum(areas, 1, colsum);
    std::default_random_engine generator;
    std::uniform_real_distribution<double> a(0.0,1.0);
    std::uniform_real_distribution<double> b(0.0,1.0);
    std::uniform_real_distribution<double> c(0.0,1.0);
    //std::cout << "w1" <<colsum.rows() << " " << colsum.cols() << std::endl;
    X.resize(n,3);
    for (int i = 0; i < X.rows(); i++){
        double rand_tri = c(generator);
        int tri_idx = binary_search_larger(colsum, rand_tri);
        auto p0 = V.row(F(tri_idx,0));
        auto p1 = V.row(F(tri_idx,1));
        auto p2 = V.row(F(tri_idx,2));
        double rand_a = a(generator);
        double rand_b = b(generator);
        if ((rand_a+rand_b) > 1){
            rand_a = 1 - rand_a;
            rand_b = 1 - rand_b;
        }
        X.row(i) = p0 + rand_a * (p1 - p0) + rand_b * (p2 - p0); 

    }
    //std::cout << "w" << std::endl;
  //for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());
}


