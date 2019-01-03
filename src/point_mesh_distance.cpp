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
  int n=X.rows(),m=FY.rows(),jj;
  D.resize(n);
  P.resize(n,3);
  N.resize(n,3);
  double d,dd;
  Eigen::RowVector3d p,pp,a,b,nn; 
  for (int i=0; i<n; i++){
  	d=99999999;
  	for (int j=0; j<m; j++){
  	  point_triangle_distance(X.row(i),VY.row(FY(j,0)),VY.row(FY(j,1)),VY.row(FY(j,2)),dd,pp);
  	  if (dd<d){
  	  	jj=j;
  	  	d=dd;
  	  	p=pp;
	  }
	}
	D(i)=d;
	P.row(i)=p;
	a=VY.row(FY(jj,0))-VY.row(FY(jj,1));
	b=VY.row(FY(jj,0))-VY.row(FY(jj,2));
	nn=a.cross(b);
	if (nn.dot(X.row(i)-p)<0) nn=-nn;
	nn=nn.normalized();
	N.row(i)=nn;
  }
}

