#include "random_points_on_mesh.h"

#include <iostream>
#include <random>

#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <assert.h>
//
// Generate n random points uniformly sampled on a given triangle mesh with
// vertex positions VX and face indices FX.
//
void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
    static std::random_device rd;
    static std::mt19937 eng( rd() ); // seed our mersenne twister
    static std::uniform_real_distribution<double> distr( 0.0, 1.0 ); // range
    
    // calculate the area of each triangle
    const int nTris = F.rows();
    Eigen::VectorXd Atv; // triangle areas, idx==Face Index
    igl::doublearea( V, F, Atv ); 

    Eigen::VectorXd C;
    // NB - we should halve the values in Atv since we used doublearea
    //      but I think we're ok if we're consistent.... TODO - verify
    Atv = Atv/Atv.sum();
    igl::cumsum( Atv, 1, C );

    using TriIdxs = std::vector<int>;
    TriIdxs randomTris( n );
    for( int i=0; i<n; ++i )
    {
        // must be an eigen way to do this...
        int t( 0 );
        double gamma = distr( eng );
        while( t<nTris && C(t)<gamma )
            ++t;

        randomTris[i] = t;
    }

    // for each random triangle, find a random place inside it.
    // Again, I should spend some time with eigen and c++XX to
    // make this prettier...sigh...
    X.resize( n, 3 );
    for( int t=0; t<n; ++t )
    {
        double alpha( distr(eng) ), beta( distr(eng) );
        if( (alpha + beta) > 1 )
        {
            // reflect back across our ||-gram
            alpha = 1 - alpha;
            beta = 1 - beta;
        }
        Eigen::RowVector3i tri( F.row( randomTris[t] ) );
        Eigen::RowVector3d v1( V.row( tri(0) ) ),
                           v2( V.row( tri(1) ) ),
                           v3( V.row( tri(2) ) );
        X.row(t) = v1 + alpha*( v2 - v1 ) + beta*( v3 - v1 );
    }

}
