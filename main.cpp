#include "random_points_on_mesh.h"
#include "hausdorff_lower_bound.h"
#include "point_to_point_rigid_matching.h"
#include "point_mesh_distance.h"
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
  const Eigen::RowVector3d orange(1.0,0.7,0.2);
  const Eigen::RowVector3d blue(0.2,0.3,0.8);
  // Load meshes
  Eigen::MatrixXd VX,VY;
  Eigen::MatrixXi FX,FY;
  igl::read_triangle_mesh((argc>1?argv[1]:"../shared/data/"),VX,FX);
  igl::read_triangle_mesh((argc>2?argv[2]:"../shared/data/"),VY,FY);

  Eigen::MatrixXd B;
  Eigen::VectorXi FI;
  const int n = 100;
  random_points_on_mesh(n,VX,FX,B,FI);
  Eigen::MatrixXd X(B.rows(),3);
  for(int x = 0;x<X.rows();x++)
  {
    X.row(x) = 
      B(x,0)*VX.row(FX(FI(x),0))+
      B(x,1)*VX.row(FX(FI(x),1))+
      B(x,2)*VX.row(FX(FI(x),2));
  }

  Eigen::MatrixXd P;
  for(int iter = 0;iter < 20;iter++)
  {
    Eigen::VectorXd D;
    point_mesh_distance(X,VY,FY,D,P);

    Eigen::Matrix3d R;
    Eigen::RowVector3d t;
    point_to_point_rigid_matching(X,P,R,t);
    VX = ((VX*R).rowwise() + t).eval();
    X = ((X*R).rowwise() + t).eval();
  }

  std::cout<<"Hausdorff from X to Y >= "<<
    hausdorff_lower_bound(n,VX,FX,VY,FY)<<std::endl;

  // Create a libigl Viewer object to toggle between point cloud and mesh
  igl::viewer::Viewer viewer;
  std::cout<<R"(
  P,p      view point cloud
  M,m      view mesh
)";

  const auto & set_meshes = [&]()
  {
    // Concatenate meshes into one big mesh
    Eigen::MatrixXd V(VX.rows()+VY.rows(),VX.cols());
    V<<VX,VY;
    Eigen::MatrixXi F(FX.rows()+FY.rows(),FX.cols());
    F<<FX,FY.array()+VX.rows();
    viewer.data.set_mesh(V,F);
    // Assign orange and blue colors to each mesh's faces
    Eigen::MatrixXd C(F.rows(),3);
    C.topLeftCorner(FX.rows(),3).rowwise() = orange;
    C.bottomLeftCorner(FY.rows(),3).rowwise() = blue;
    viewer.data.set_colors(C);
  };
  const auto & set_points = [&]()
  {
    Eigen::MatrixXd XP(X.rows()+P.rows(),3);
    XP<<X,P;
    Eigen::MatrixXd C(XP.rows(),3);
    C.array().topRows(X.rows()).rowwise() = (1.-(1.-orange.array())*.8);
    C.array().bottomRows(P.rows()).rowwise() = (1.-(1.-blue.array())*.8);
    viewer.data.set_points(XP,C);
  };
  set_meshes();
  set_points();

  viewer.core.point_size = 3;
  viewer.launch();

  return EXIT_SUCCESS;
}

