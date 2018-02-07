#include "random_points_on_mesh.h"
#include <iostream>
#include <igl/doublearea.h>
#include <igl/cumsum.h>

int bs(const Eigen::VectorXd & SumArea, double x)
{
  int A = 0, B = SumArea.rows() - 1;
  while (A <= B)
  {
    int C = (A+B)/2;
    if (SumArea(C) > x){
      B = C - 1;
    }
    else{
      A = C + 1;
    }
  }
  return B + 1;
}


void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
    X.resize(n,3);

    Eigen::MatrixXd rand = Eigen::VectorXd::Random(n);
    rand = (rand + Eigen::VectorXd::Constant(n,1)) * .5;


    Eigen::VectorXd Areas(F.rows()); 
    Eigen::VectorXd SumArea(F.rows()); 
    
    igl::doublearea(V,F,Areas);
    igl::cumsum(Areas, 1, SumArea);
    SumArea /= SumArea.maxCoeff();



    for(int i = 0;i<n;i++) {
        int index = bs(SumArea, rand(i));

        if (index > (SumArea.rows() - 1)){
          index = SumArea.rows() - 1;
        }

        Eigen::MatrixXd r = Eigen::VectorXd::Random(2);
        r = (r + Eigen::VectorXd::Constant(2,1)) * .5;

        if (r(0) + r(1) > 1){
          r(0) = 1-r(0);
          r(1) = 1-r(1);
        }
        X.row(i) = V.row(F(index, 0)) + (r(0)*(V.row(F(index, 1)) - V.row(F(index, 0)))) + (r(1)*(V.row(F(index, 2)) - V.row(F(index, 0))));
    }


}

