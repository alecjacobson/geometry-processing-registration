#include "point_triangle_distance.h"
#include <Eigen/Dense>
#include <iostream>
#define ESP 1E-5
//using namespace Eigen;
void point_triangle_distance(
  const Eigen::RowVector3d & P0,//x
  const Eigen::RowVector3d & P1,//a
  const Eigen::RowVector3d & P2,//b
  const Eigen::RowVector3d & P3,//c
  double & d,
  Eigen::RowVector3d & p)
{
  typedef Eigen::RowVector3d Vector;
  // Replace with your code
  d = 0;
  p = P1;
  //from http://cs.swan.ac.uk/~csmark/PDFS/1995_3D_distance_point_to_triangle
  //Let a be the origin of the plane
  //The two basis are:

  Vector P1P0 = P0 - P1; Vector P0P1 = -P1P0;
  Vector P2P0 = P0 - P2; Vector P0P2 = -P2P0;
  Vector P3P0 = P0 - P3; Vector P0P3 = -P3P0;

  Vector P1P2 = P2 - P1; Vector P2P1 = -P1P2;
  Vector P1P3 = P3 - P1; Vector P3P1 = -P1P3;
  Vector P3P2 = P2 - P3; Vector P2P3 = -P3P2;

  Vector Np = P1P2.cross(P1P3); //eq1
                                
  double cos_alpha = P1P0.dot(Np) / (P1P0.norm()*Np.norm());//eq2
                                                            
  double P0P0p_norm = P0P1.norm()*cos_alpha;//eq3
                                            
  Vector P0P0p = -P0P0p_norm*(Np / Np.norm()); //eq4

  Vector P0p = P0 + P0P0p;//eq5

  Vector V1 = P2P1 / P2P1.norm() + P3P1 / P3P1.norm();//eq6
  Vector V2 = P3P2 / P3P2.norm() + P1P2 / P1P2.norm();//eq6
  Vector V3 = P1P3 / P1P3.norm() + P2P3 / P2P3.norm();//eq6

  Vector P1P0p = P0p - P1; Vector P0pP1 = -P1P0p;
  Vector P2P0p = P0p - P2; Vector P0pP2 = -P2P0p;
  Vector P3P0p = P0p - P3; Vector P0pP3 = -P3P0p;

  double f1 = V1.cross(P1P0p).dot(Np);
  double f2 = V2.cross(P2P0p).dot(Np);
  double f3 = V3.cross(P3P0p).dot(Np);
  
  if (f2 < 0 && f1>0)//If P0' is clockwise of V2 and anticlockwise of V1
  {
    if (P0pP1.cross(P0pP2).dot(Np) < 0)//eq7//outside of the triangle
    {
      Vector R = P0pP2.cross(P0pP1).cross(P1P2);//eq8
      double cos_gamma = P0pP1.dot(R) / (P0pP1.norm()*R.norm());//eq9
      double P0pP0pp_norm = P0pP1.norm()*cos_gamma;//eq10
      Vector P0pP0pp = P0pP0pp_norm*R / R.norm();//eq11
      Vector P0pp = P0p + P0pP0pp;//eq12
      Vector num = P0pp - P1; Vector denom = P2 - P1;
      double t = 0;
      for (int i = 0; i < 3; i++)
        if (abs(num(i)) > ESP && abs(denom(i)) > ESP)
          t += num(i) / denom(i);
      t /= 3;
      if (t >= 0 && t <= 1)
      {
        d = sqrt(P0pP0pp_norm*P0pP0pp_norm + P0P0p_norm*P0P0p_norm);
        p = P0pp;
      }
      else if (t < 0)
      {
        d = P0P1.norm();
        p = P1;
      }
      else if (t > 1)
      {
        d = P0P2.norm();
        p = P2;
      }
    }
    else
    {
      d = P0P0p_norm;
      p = P0p;
    }
    return;
  }

  

  
  if (f3 < 0 && f2>0)//If P0' is clockwise of V3 and anticlockwise of V2
  {
    if (P0pP2.cross(P0pP3).dot(Np) < 0)//eq7//outside of the triangle
    {
      auto R = P0pP3.cross(P0pP2).cross(P2P3);//eq8
      
      
      double cos_gamma = P0pP2.dot(R) / (P0pP2.norm()*R.norm());//eq9

      auto P0pP0pp_norm = P0pP2.norm()*cos_gamma;//eq10

      auto P0pP0pp = P0pP0pp_norm*R / R.norm();//eq11
      auto P0pp = P0p + P0pP0pp;//eq12
      auto num = P0pp - P2; auto denom = P3 - P2;
      double t = 0;
      for (int i = 0; i < 3; i++)
        if (abs(num(i)) > ESP && abs(denom(i)) > ESP)
          t += num(i) / denom(i);
      t /= 3;
      if (t >= 0 && t <= 1)
      {
        d = sqrt(P0pP0pp_norm*P0pP0pp_norm + P0P0p_norm*P0P0p_norm);
        p = P0pp;
      }
      else if (t < 0)
      {
        d = P0P2.norm();
        p = P2;
      }
      else if (t > 1)
      {
        d = P0P3.norm();
        p = P3;
      }
    }
    else
    {
      d = P0P0p_norm;
      p = P0p;
    }
    return;
  }


  if (f1 < 0 && f3>0)//If P0' is clockwise of V1 and anticlockwise of V3
  {
    if (P0pP3.cross(P0pP1).dot(Np) < 0)//eq7//outside of the triangle
    {
      auto R = P0pP1.cross(P0pP3).cross(P3P1);//eq8


      double cos_gamma = P0pP3.dot(R) / (P0pP3.norm()*R.norm());//eq9

      auto P0pP0pp_norm = P0pP3.norm()*cos_gamma;//eq10

      auto P0pP0pp = P0pP0pp_norm*R / R.norm();//eq11
      auto P0pp = P0p + P0pP0pp;//eq12
      auto num = P0pp - P3; auto denom = P1 - P3;
      double t = 0;
      for (int i = 0; i < 3; i++)
        if (abs(num(i)) > ESP && abs(denom(i)) > ESP)
          t += num(i) / denom(i);
      t /= 3;
      if (t >= 0 && t <= 1)
      {
        d = sqrt(P0pP0pp_norm*P0pP0pp_norm + P0P0p_norm*P0P0p_norm);
        p = P0pp;
      }
      else if (t < 0)
      {
        d = P0P3.norm();
        p = P3;
      }
      else if (t > 1)
      {
        d = P0P1.norm();
        p = P1;
      }
    }
    else
    {
      d = P0P0p_norm;
      p = P0p;
    }
    return;
  }
  else
  {
    std::cout << "WTF f1:" << f1 << " f2: " << f2 << " f3: " << f3 << std::endl;
  }
   
 
}
