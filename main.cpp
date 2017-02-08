#include "hausdorff_lower_bound.h"
#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
// testing
#include "random_points_on_mesh.h"
#include "closest_rotation.h"
#include "point_to_point_rigid_matching.h"
#include <math.h>




void
testSimpleMesh_extra( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    Eigen::MatrixXd X, Y;   // vertices
    Eigen::MatrixXi TX, TY; // triangles
    // for reading in the reduced1/2/3 cases...
    igl::read_triangle_mesh(
        ("../shared/data/max-registration-partial.obj"),X,TX);//OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/max-registration-complete.obj"),Y,TY);//VY,FY);

    std::cout<<"blah"<<std::endl;
    OVX.resize(3,3);
    VY.resize(3,3);
    FX.resize(1,3);
    FY.resize(1,3);
   
    // Test only triangle X -> Y
    //X triangle 24900 --->  3703 20927 20909
    //Y triangle  3276 (1106, 1107,  723)
    const int triX( 24900 ), triY( 3276 );
    for( int i=0; i<3; ++i )
    {
        OVX.row(i) = X.row( TX.row( triX )( i ) );
        VY.row(i) = Y.row( TY.row( triY)( i ) );
    }

    std::cout << "Vertices are OVX and then OVY:"<<std::endl;
    std::cout<<"OVX:"<<std::endl;
    std::cout << OVX<<std::endl;
    std::cout<<"VY:"<<std::endl;    
    std::cout<<VY<<std::endl;

    FX.row(0) = Eigen::RowVector3i( 0, 1, 2 );
    FY.row(0) = Eigen::RowVector3i( 0, 1, 2 );

    std::cout<<"FX:"<<std::endl;
    std::cout<<FX<<std::endl;
    std::cout<<"FY:"<<std::endl;    
    std::cout<<FY<<std::endl;
}

void
testSimpleMesh_zone0( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    OVX = VY;
    for( int i=0; i<OVX.rows(); ++i )
        OVX.row(i) += Eigen::Vector3d( 0, 0, 3 );
}


// 4 is wrong
void
testSimpleMesh_zone4( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    OVX = VY;
    for( int i=0; i<OVX.rows(); ++i )
        OVX.row(i) += Eigen::Vector3d( 1, 0, 0 );
    
    // let's rotate OVX by 1 radian, test region 3
    Eigen::Affine3d rot;
    rot = Eigen::AngleAxisd( -M_PI, Eigen::Vector3d( 0, 0, 1 ) );
    OVX.transpose() = ( rot ).matrix() * OVX.transpose();
}


// Zone 5 is wrong :(
void
testSimpleMesh_zone5( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    OVX = VY;
    for( int i=0; i<OVX.rows(); ++i )
        OVX.row(i) += Eigen::Vector3d( .6, 0, 0 );
    
    // let's rotate OVX by 1 radian, test region 3
    Eigen::Affine3d rot;
    rot = Eigen::AngleAxisd( -M_PI/2, Eigen::Vector3d( 0, 0, 1 ) );
    OVX.transpose() = ( rot ).matrix() * OVX.transpose();
}
void
testSimpleMesh_zone6( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    OVX = VY;
    for( int i=0; i<OVX.rows(); ++i )
        OVX.row(i) += Eigen::Vector3d( 3, 0, 0 );
    
    // let's rotate OVX by 1 radian, test region 3
    Eigen::Affine3d rot;
    rot = Eigen::AngleAxisd( -M_PI/3, Eigen::Vector3d( 0, 0, 1 ) );
    OVX.transpose() = ( rot ).matrix() * OVX.transpose();
}
void
testSimpleMesh_zone2( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    OVX = VY;
    for( int i=0; i<OVX.rows(); ++i )
        OVX.row(i) += Eigen::Vector3d( 3, 0, 0 );
    
    // let's rotate OVX by 1 radian, test region 3
    Eigen::Affine3d rot;
    rot = Eigen::AngleAxisd( M_PI/3, Eigen::Vector3d( 0, 0, 1 ) );
    OVX.transpose() = ( rot ).matrix() * OVX.transpose();
}


void
testSimpleMesh_zone3( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);
    
    // let's rotate OVX by 1 radian, test region 3
    Eigen::Affine3d rot, trans;
    rot = Eigen::AngleAxisd( 1, Eigen::Vector3d( 0, 0, 1 ) );
    trans = Eigen::Translation3d( Eigen::Vector3d( 1.1, 0, 0 ) );
    OVX.transpose() = ( trans * rot ).matrix() * VY.transpose();
}

void
testSimpleMesh_zone1( Eigen::MatrixXd &OVX,
                      Eigen::MatrixXi &FX,
                      Eigen::MatrixXd &VY,
                      Eigen::MatrixXi &FY )
{
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri-offset.obj"),OVX,FX);
    igl::read_triangle_mesh(
        ("../shared/data/simple-x-tri.obj"),VY,FY);

    // OVX = VY;
    // for( int i=0; i<OVX.rows(); ++i )
    //     OVX.row(i) += Eigen::Vector3d( 3, 0, 0 );
    
    // // let's rotate OVX by 1 radian, test region 3
    // Eigen::Affine3d rot;
    // rot = Eigen::AngleAxisd( M_PI/3, Eigen::Vector3d( 0, 0, 1 ) );
    // OVX.transpose() = ( rot ).matrix() * OVX.transpose();

}


int main(int argc, char *argv[])
{
  static bool doTests( false );
  static bool freezeFirstFrame( false );
  
  // Load input meshes
  Eigen::MatrixXd OVX,VX,VY;
  Eigen::MatrixXi FX,FY;

  if( doTests )
  {
      //testSimpleMesh_zone0( OVX, FX, VY, FY );
      //testSimpleMesh_zone1( OVX, FX, VY, FY );
      //testSimpleMesh_zone2( OVX, FX, VY, FY );
      //testSimpleMesh_zone3( OVX, FX, VY, FY );
      //testSimpleMesh_zone4( OVX, FX, VY, FY ); // iffy...
      //testSimpleMesh_zone5( OVX, FX, VY, FY ); // fixed!
      //testSimpleMesh_zone6( OVX, FX, VY, FY );
      testSimpleMesh_extra( OVX, FX, VY, FY ); // passes...
  }
  else
  {
      igl::read_triangle_mesh(
          (argc>1?argv[1]:"../shared/data/max-registration-partial.obj"),OVX,
          FX);
      igl::read_triangle_mesh(
          (argc>2?argv[2]:"../shared/data/max-registration-complete.obj"),VY,
          FY);

      // Eigen::MatrixXd x,y;
      // Eigen::Matrix3d R;
      // Eigen::RowVector3d t;

      // x = OVX.block(0,0,3,3);
      // y = VY.block(0,0,3,3);
      // point_to_point_rigid_matching( x, y, R, t );
      // return 0;
      
  }
  
  int num_samples = 100; // TODO reset to 100
  bool show_samples = true;
  ICPMethod method = ICP_METHOD_POINT_TO_PLANE;//ICP_METHOD_POINT_TO_POINT;

  // testing
  // Eigen::MatrixXd X;
  // random_points_on_mesh( 5, OVX, FX, X );
  // return 0;
  
  // Create a libigl Viewer object to toggle between point cloud and mesh
  igl::viewer::Viewer viewer;
  std::cout<<R"(
  [space]  toggle animation
  H,h      print lower bound on directed Hausdorff distance from X to Y
  M,m      toggle between point-to-point and point-to-plane methods
  R,r      reset, also recomputes a random sampling and closest points
  P,p      show sample points
  S        double number of samples
  s        halve number of samples
)";

  // predefined colors
  const Eigen::RowVector3d orange(1.0,0.7,0.2);
  const Eigen::RowVector3d blue(0.2,0.3,0.8);


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
    // if( freezeFirstFrame ) {
    // static Eigen::MatrixXd X,P;
    // static bool once(true);
    // if( once )
    // {
    //     random_points_on_mesh(num_samples,VX,FX,X);
    //     once = false;
    // }
    //} else {
    Eigen::MatrixXd X,P;
    random_points_on_mesh(num_samples,VX,FX,X);
    

    Eigen::VectorXd D;
    Eigen::MatrixXd N;
    point_mesh_distance(X,VY,FY,D,P,N);

    Eigen::MatrixXd XP(X.rows()+P.rows(),3);
    XP<<X,P;

    Eigen::MatrixXd C(XP.rows(),3);
    C.array().topRows(X.rows()).rowwise() = (1.-(1.-orange.array())*.8);
    C.array().bottomRows(P.rows()).rowwise() = (1.-(1.-blue.array())*.4);
    viewer.data.set_points(XP,C);

    Eigen::MatrixXi E(X.rows(),2);
    E.col(0) = Eigen::VectorXi::LinSpaced(X.rows(),0,X.rows()-1);
    E.col(1) = Eigen::VectorXi::LinSpaced(X.rows(),X.rows(),2*X.rows()-1);

    viewer.data.set_edges(XP,E,Eigen::RowVector3d(0.3,0.3,0.3));
  };


  const auto & reset = [&]()
  {
    VX = OVX;
    set_meshes();
  };

  viewer.callback_pre_draw = [&](igl::viewer::Viewer &)->bool
  {
    static bool first( true );
    if(viewer.core.is_animating || first )
    {
      ////////////////////////////////////////////////////////////////////////
      // Perform single iteration of ICP method
      ////////////////////////////////////////////////////////////////////////
      Eigen::Matrix3d R;
      Eigen::RowVector3d t;
      icp_single_iteration(VX,FX,VY,FY,num_samples,method,R,t);
      // Apply transformation to source mesh
      VX = ((VX*R).rowwise() + t).eval();
      set_meshes();
      if(show_samples)
      {
        set_points();
      }
      first = false;
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
        break;
      case 'R':
      case 'r':
        reset();
        if(show_samples) set_points();
        break;
      case 'S':
        num_samples = (num_samples-1)*2;
        break;
      case 's':
        num_samples = (num_samples/2)+1;
        break;
      default:
        return false;
    }
    return true;
  };

  reset();
  viewer.core.is_animating = false; // start with it off...
  viewer.core.point_size = 10;
  viewer.launch();

  return EXIT_SUCCESS;
}



    // trans = Eigen::Matrix4d::Identity();
    // trans.block<1,3>(0,3) = Eigen::Vector3d( 3, 0, 0 );
