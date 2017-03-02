#include "point_mesh_distance.h"
#include "point_triangle_distance.h"

#include <iostream>
#include <vector>
#include <igl/per_face_normals.h>
#include <cmath>

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
    static bool debug( false );
    
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
    if( debug )
        std::cout << "Have " << X.rows() << " points with " << nTris <<
            " tris" << std::endl;
    
    const int nXs = X.rows();
    std::vector<int> closestTris( nXs );
    double d( 0 );
    Eigen::RowVector3d p;

    // do a first pass to set the initial distances...
    for( int i=0; i<nXs; ++i )
    {
        Eigen::RowVector3d x( X.row(i) );

        for( int t=0; t<1; ++t )
        {
            Eigen::RowVector3i tri( FY.row(t) );
            if( debug )
            {
                std::cout <<"--------------------------"<<std::endl;
                std::cout << FY.row(t) << std::endl;
                std::cout << tri(0)<<","<<tri(1)<<","<<tri(2)<<std::endl;
            }
            point_triangle_distance( x,
                                     VY.row( tri(0) ),
                                     VY.row( tri(1) ),
                                     VY.row( tri(2) ),
                                     d,
                                     p );
            d = std::abs( d );
            // set the D this time, but just for the first triangle
            D(i) = d;
            P.row(i) = p;
            closestTris[i] = t;
            if( debug )
                std::cout <<"init value d= "<<d<<" for p "<<p<<std::endl;
        }
    }

    // now find the closest...
    int candidates( 0 );
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
            d = std::abs( d );
            if( d < D(i) ) // save new shortest distance...
            {
                D(i) = d;
                if( debug )
                    std::cout<<"SAVED! d is "<<d<<" for tri "<<t<<" in pos "<<
                        i<<std::endl;
                P.row(i) = p;
                if( debug )
                    std::cout<<"Storing p "<<p<<" into P.row("<<i<<")= "<<
                        P.row(i)<<std::endl;

                closestTris[i] = t;
                ++candidates;                
            }
        }
    }

    if( debug )
    {
        std::cout << "Looked at " << nTris <<" and considered n candidates "<<
            candidates << " triangles " << std::endl;
        std::cout << "Point " << X.row(0) << " closest to tri " <<
            closestTris[0] << " = " << FY.row(closestTris[0]) <<std::endl;
        std::cout << "Debugging ptd for it by calling again..." << std::endl;
    }
    
    // copy the normals for those distances/triangles here
    // note the assumption in the instructions that it's "ok" to
    // assume that all the points are inside triangles, presumably
    // to avoid normal interpolation at vertices and edges
    for( int i=0; i<nXs; ++i )
        N.row(i) = Normals.row( closestTris[i] );
}
