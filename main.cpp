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
  Eigen::MatrixXd X = VX;
  Eigen::MatrixXd P = VY;

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

