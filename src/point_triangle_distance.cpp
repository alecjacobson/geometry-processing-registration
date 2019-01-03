#include "point_triangle_distance.h"
#include <fstream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  Eigen::RowVector3d n=b-a,tp=c-a,x0;
  n=n.cross(tp).normalized();
  if (n.dot(x-a)<0) n=-n;
  x0=x-n.dot(x-a)*n;
  Eigen::RowVector3d ab=b-a,bc=c-b,ca=a-c;
  ab=ab.normalized();
  bc=bc.normalized();
  ca=ca.normalized();
  Eigen::RowVector3d a2b=a+ab*ab.dot(x0-a),b2c=b+bc*bc.dot(x0-b),c2a=c+ca*ca.dot(x0-c);
  double dd;
  d=(x-a).norm();
  p=a;
  dd=(x-b).norm();
  if (dd<d){
  	d=dd;
  	p=b;
  }
  dd=(x-c).norm();
  if (dd<d){
  	d=dd;
  	p=c;
  }
  dd=(x-a2b).norm();
  if (((a2b-a).dot(a2b-b)<0)&&(dd<d)){
  	d=dd;
  	p=a2b;
  }
  dd=(x-b2c).norm();
  if (((b2c-b).dot(b2c-c)<0)&&(dd<d)){
  	d=dd;
  	p=b2c;
  }
  dd=(x-c2a).norm();
  if (((c2a-a).dot(c2a-c)<0)&&(dd<d)){
  	d=dd;
  	p=c2a;
  }
}

