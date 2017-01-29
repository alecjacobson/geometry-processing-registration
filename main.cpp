#include "hausdorff_lower_bound.h"
#include "icp.h"
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
  Eigen::MatrixXd OVX,VX,VY;
  Eigen::MatrixXi FX,FY;
  igl::read_triangle_mesh(
    (argc>1?argv[1]:"../shared/data/max-face-unaligned.obj"),OVX,FX);
  igl::read_triangle_mesh(
    (argc>2?argv[2]:"../shared/data/max-low-res.obj"),VY,FY);

  const int num_samples = 100;
  const ICPMethod method = ICP_METHOD_POINT_TO_POINT;
  Eigen::MatrixXd X,P;
  Eigen::Matrix3d R;
  Eigen::RowVector3d t;

  // Create a libigl Viewer object to toggle between point cloud and mesh
  igl::viewer::Viewer viewer;
  std::cout<<R"(
[space]  toggle animation
R,r      reset
H,h      print lower bound on directed Hausdorff distance from X to Y
)";

  const auto & set_meshes = [&]()
  {
    // Concatenate meshes into one big mesh
    Eigen::MatrixXd V(VX.rows()+VY.rows(),VX.cols());
    V << VX, VY;
    Eigen::MatrixXi F(FX.rows()+FY.rows(),FX.cols());
    F<<FX,FY.array()+VX.rows();
    viewer.data.clear();
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
  const auto & reset = [&]()
  {
    VX = OVX;
    R = Eigen::Matrix3d::Identity();
    t = Eigen::RowVector3d::Zero();
    set_meshes();
    set_points();
  };

  viewer.callback_pre_draw = [&](igl::viewer::Viewer &)->bool
  {
    if(viewer.core.is_animating)
    {
      icp(VX,FX,VY,FY,num_samples,1,method,R,t,X,P);
      VX = ((VX*R).rowwise() + t).eval();
      set_meshes();
      set_points();
    }
    return false;
  };
  viewer.callback_key_pressed = 
    [&](igl::viewer::Viewer &,unsigned char key,int)->bool
  {
    switch(key)
    {
      case ' ':
        viewer.core.is_animating ^= 1;
        break;
      case 'R':
      case 'r':
        reset();
        break;
      case 'H':
      case 'h':
        std::cout<<"D_{H}(X -> Y) >= "<<
          hausdorff_lower_bound(VX,FX,VY,FY,num_samples)<<std::endl;
        break;
      default:
        return false;
    }
    return true;
  };

  reset();
  viewer.core.is_animating = true;
  viewer.core.point_size = 3;
  viewer.launch();

  return EXIT_SUCCESS;
}

