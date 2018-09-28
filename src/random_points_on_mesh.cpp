#include "random_points_on_mesh.h"
#include <igl/doublearea.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  double a,b,Ssum,cho;
  Eigen::MatrixXd S(F.rows(),1);
  Eigen::Vector3d p,a1,b1,st;
  int j;
  for (int i=0; i<n; i++){
  	igl::doublearea(V,F,S);
  	Ssum=S.sum();
  	cho=double(rand()%10000)/10001*Ssum;
  	for (j=0; j<F.rows(); j++)
  	  if (cho<S(j,0)) break;
  	  else cho-=S(j,0);
  	a=double(rand()%10000)/10001;
  	b=double(rand()%10000)/10001;
  	if (a+b>1){
  		a=1-a;
  		b=1-b;
	}
	X.row(i)=V.row(F(j,0))+a*(V.row(F(j,1))-V.row(F(j,0)))+b*(V.row(F(j,2))-V.row(F(j,0))); 
  }
}

