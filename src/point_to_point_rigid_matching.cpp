#include "point_to_point_rigid_matching.h"
#include <Eigen/Dense>
#include "closest_rotation.h"
#include <fstream>
#include <Eigen/SVD>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
/*
  Eigen::VectorXd Xavg=X.colwise().sum()/n;
  Eigen::VectorXd Pavg=P.colwise().sum()/n;
  Eigen::MatrixXd ones=Eigen::MatrixXd::Ones(n,3),X2,X1=X,P1=P;
  printf("%u %u ",(ones*Xavg.asDiagonal()).rows(),(ones*Xavg.asDiagonal()).cols());
  X1=X1-ones*Xavg.asDiagonal();
  P1=P1-ones*Pavg.asDiagonal();
  //X2=X1.transpose();
  printf("aaa");
  Eigen::MatrixXd W=Eigen::MatrixXd::Zero(3,3);
  W=P1.transpose()*X1;
  printf("bbb");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d U=svd.matrixU();
  Eigen::Matrix3d V=svd.matrixV();
  R=U*V.transpose();
  closest_rotation(W,R);
  printf("ccc");
  t=-Xavg+R*Pavg;*/
  int n=X.rows();
  Eigen::MatrixXd X1=X.col(0),X2=X.col(1),X3=X.col(2);
  Eigen::MatrixXd P1=P.col(0),P2=P.col(1),P3=P.col(2);
  Eigen::MatrixXd one=Eigen::MatrixXd::Ones(n,1);
  Eigen::MatrixXd zero=Eigen::MatrixXd::Zero(n,1);
  Eigen::MatrixXd row1,row2,row3;
  row1.resize(n,6);
  row2.resize(n,6);
  row3.resize(n,6);
  row1 << zero,X3,-X2,one,zero,zero;
  row2 << -X3,zero,X1,zero,one,zero;
  row3 << X2,-X1,zero,zero,zero,one;
  Eigen::MatrixXd A,u,AA,b;
  A.resize(3*n,6);
  A << row1,
       row2,
	   row3;
  P1=P1-X1;
  P2=P2-X2;
  P3=P3-X3;
  b.resize(3*n,1);
  b << P1,
       P2,
       P3;
  AA=A.transpose()*A;
  u=AA.inverse()*A.transpose()*b;
  Eigen::Matrix3d M=Eigen::Matrix3d::Identity();
  double alpha=u(0,0),beta=u(1,0),gamma=u(2,0);
  M(0,1)=-gamma;
  M(0,2)=beta;
  M(1,0)=gamma;
  M(1,2)=-alpha;
  M(2,0)=-beta;
  M(2,1)=alpha;
  closest_rotation(M,R);
  t = Eigen::RowVector3d(u(3,0),u(4,0),u(5,0));
}

