#include "hausdorff_lower_bound.h"
#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
  // Load input meshes
  Eigen::MatrixXd OVX,VX,VY;
  Eigen::MatrixXi FX,FY;
  igl::read_triangle_mesh(
    (argc>1?argv[1]:"../data/max-registration-partial.obj"),OVX,FX);
  igl::read_triangle_mesh(
    (argc>2?argv[2]:"../data/max-registration-complete.obj"),VY,FY);

  int num_samples = 100;
  bool show_samples = true;
  ICPMethod method = ICP_METHOD_POINT_TO_POINT;

  igl::opengl::glfw::Viewer viewer;
  const int xid = viewer.selected_data_index;
  viewer.append_mesh();
  const int yid = viewer.selected_data_index;

  std::cout<<R"(
  [space]  toggle animation
  H,h      print lower bound on directed Hausdorff distance from X to Y
  M,m      toggle between point-to-point and point-to-plane methods
  P,p      show sample points
  R,r      reset, also recomputes a random sampling and closest points
  S        double number of samples
  s        halve number of samples
)";

  // predefined colors
  const Eigen::RowVector3d orange(1.0,0.7,0.2);
  const Eigen::RowVector3d blue(0.2,0.3,0.8);
  const auto & set_points = [&]()
  {
    if(show_samples)
    {
      Eigen::MatrixXd X,P;
      random_points_on_mesh(num_samples,VX,FX,X);
      viewer.data_list[xid].set_points(X,(1.-(1.-orange.array())*.8));
      Eigen::VectorXd D;
      Eigen::MatrixXd N;
      point_mesh_distance(X,VY,FY,D,P,N);
      viewer.data_list[yid].set_points(P,(1.-(1.-blue.array())*.4));
      Eigen::MatrixXi E(X.rows(),2);
      E.col(0) = Eigen::VectorXi::LinSpaced(X.rows(),0,X.rows()-1);
      E.col(1) = Eigen::VectorXi::LinSpaced(X.rows(),X.rows(),2*X.rows()-1);
      Eigen::MatrixXd XP(X.rows()+P.rows(),3);
      XP<<X,P;
      viewer.data_list[xid].set_edges(XP,E,Eigen::RowVector3d(0.3,0.3,0.3));
    }else
    {
      viewer.data_list[xid].clear_points();
      viewer.data_list[xid].clear_edges();
      viewer.data_list[yid].clear_points();
    }
  };
  const auto & reset = [&]()
  {
    VX = OVX;
    viewer.data_list[xid].set_vertices(VX);
    viewer.data_list[xid].compute_normals();
    set_points();
  };
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &)->bool
  {
    if(viewer.core().is_animating)
    {
      ////////////////////////////////////////////////////////////////////////
      // Perform single iteration of ICP method
      ////////////////////////////////////////////////////////////////////////
      Eigen::Matrix3d R;
      Eigen::RowVector3d t;
      icp_single_iteration(VX,FX,VY,FY,num_samples,method,R,t);
      // Apply transformation to source mesh
      VX = ((VX*R).rowwise() + t).eval();
      viewer.data_list[xid].set_vertices(VX);
      viewer.data_list[xid].compute_normals();
      set_points();
    }
    return false;
  };
  viewer.callback_key_pressed = 
    [&](igl::opengl::glfw::Viewer &,unsigned char key,int)->bool
  {
    switch(key)
    {
      case ' ':
        viewer.core().is_animating ^= 1;
        break;
      case 'H':
      case 'h':
        std::cout<<"D_{H}(X -> Y) >= "<<
          hausdorff_lower_bound(VX,FX,VY,FY,num_samples)<<std::endl;
        break;
      case 'M':
      case 'm':
        method = (ICPMethod)((((int)method)+1)%((int)NUM_ICP_METHODS));
        std::cout<< "point-to-"<<
          (method==ICP_METHOD_POINT_TO_PLANE?"plane":"point")<<std::endl;
        break;
      case 'P':
      case 'p':
        show_samples ^= 1;
        set_points();
        break;
      case 'R':
      case 'r':
        reset();
        break;
      case 'S':
        num_samples = (num_samples-1)*2;
        set_points();
        break;
      case 's':
        num_samples = (num_samples/2)+1;
        set_points();
        break;
      default:
        return false;
    }
    return true;
  };

  viewer.data_list[xid].set_mesh(OVX,FX);
  viewer.data_list[xid].set_colors(orange);
  viewer.data_list[yid].set_mesh(VY,FY);
  viewer.data_list[yid].set_colors(blue);
  reset();
  viewer.core().is_animating = false;
  viewer.data().point_size = 10;
  viewer.launch();

  return EXIT_SUCCESS;
}
