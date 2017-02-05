#include "point_mesh_distance.h"
#include "point_triangle_distance.h"

#include <iostream>
#include <vector>
#include <igl/per_face_normals.h>


/*
Compute the distances D between a set of given points X and their closest points P on a given mesh with vertex positions VY and face indices FY. For each point in P also output a corresponding normal in N.

It is OK to assume that all points in P lie inside (rather than exactly at vertices or exactly along edges) for the purposes of normal computation in N.
*/

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    // Replace with your code
    D.resize( X.rows() );
    P.resizeLike( X );
    N.resizeLike( X );

    // only have to compute normals once this way...ish...ptd does too...
    // we can assume that all points P lie inside triangles...
    Eigen::MatrixXd Normals( FY.rows(), 3);
   
    igl::per_face_normals( VY, FY, Eigen::Vector3d(1,1,1).normalized(),
                           Normals );
    
    // start with a bad implementation and make it better
    // later...

    // exhaustively walk all the triangles and find the closest
    // point....ugh...should use a hierachical spatial data structure
    // like an octree or something to limit the number of triangles
    // by finding the closest cells and only walk their triangles.
    
    const int nTris = FY.rows();
    const int nXs = X.rows();
    std::vector<int> closestTris( nXs );
    double d( 0 );
    Eigen::RowVector3d p;
    for( int i=0; i<nXs; ++i )
    {
        Eigen::RowVector3d x( X.row(i) );

        for( int t=0; t<nTris; ++t )
        {
            Eigen::RowVector3i tri( FY.row(t) );
            point_triangle_distance( x,
                                     VY.row( tri(0) ),
                                     VY.row( tri(1) ),
                                     VY.row( tri(2) ),
                                     d,
                                     p );
            if( d < D(i) ) // save new shortest distance...
            {
                D(i) = d;
                P.row(i) = p;
                closestTris[i] = t;
            }
        }
    }

    // copy the normals for those distances/triangles here
    // note the assumption the instructions that it's "ok" to
    // assume that all the points are inside triangles, presumably
    // to avoid normal interpolation at vertices and edges
    
    for( int i=0; i<nXs; ++i )
        N.row(i) = Normals.row( closestTris[i] );
}




    // std::cout << "X is: " << X.rows() << "x" << X.cols() << std::endl;
    // std::cout << "D is: " << D.rows() << "x" << D.cols() << std::endl;
    // std::cout << "N is: " << N.rows() << "x" << N.cols() << std::endl;
    // std::cout << "FY is: " << FY.rows() << "x" << FY.cols() << std::endl;
    // std::cout << "VY is: " << VY.rows() << "x" << VY.cols() << std::endl;
    // std::cout << "Normals is: " << Normals.rows() << "x" << Normals.cols()
    // << std::endl;


    // N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    // for(int i = 0;i<X.rows();i++)
    //     P.row(i) = VY.row(i%VY.rows());
    // D = (X-P).rowwise().norm();

    //std::cout << "our normals are..." << Normals << std::endl;
    //return;
