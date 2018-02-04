#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
#include "closest_rotation.h"
#include <Eigen/Sparse>
using namespace Eigen;

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    
    typedef Eigen::Triplet<double> T;
    std::vector<T> tripletList;
    tripletList.reserve(X.rows()*3);
    
  //Compute B
    int n = X.rows();
    Eigen::MatrixXd B, B_new;
    B = Eigen::MatrixXd::Zero(3*n,1);
    for (int i = 0; i < 3; i ++) {
        B.middleRows(i*n, n).array() = (X.col(i).array() - P.col(i).array());
    }
    
    //Compute A (in the assignment)
    Eigen::MatrixXd A, C1,M;
    A = Eigen::MatrixXd::Zero(3*n, 6);
    for (int i = 3; i < 6; i ++) {
        A.block((i-3)*n,i,n,1) = Eigen::MatrixXd::Ones(n,1);
    }
    
    A.block(n,0,n,1) = -X.col(2);
    A.block(2*n,0,n,1) = X.col(1);
    
    A.block(0,1,n,1) = X.col(2);
    A.block(2*n,1,n,1) = -X.col(0);
    
    A.block(0,2,n,1) = -X.col(1);
    A.block(n,2,n,1) = X.col(0);
    
    //Compute C for which CX = B;
    //Create sparse matrix
    Eigen::SparseMatrix<double> mat(n, 3*n);
    for(int i = 0; i < N.rows(); i ++)
    {
        tripletList.push_back(T(i,i,N(i,0)));
        tripletList.push_back(T(i,n+i,N(i,1)));
        tripletList.push_back(T(i,2*n + i,N(i,2)));
    }
    
    
    mat.setFromTriplets(tripletList.begin(), tripletList.end());
    
    C1 = mat * A;
    B_new = mat * B;
    
    Eigen::MatrixXd u;
    //Solves the system of equations formulated as LS
    u = C1.jacobiSvd(ComputeThinU | ComputeThinV).solve(-B_new);
    

   
    M = Eigen::Matrix3d::Identity();

    //Extract the translation
    t(0) = u(3,0);
    t(1) = u(4,0);
    t(2) = u(5,0);
    
    //Coarse rotation estimation
    M(1,0) = -u(2,0);
    M(2,0) = u(1,0);
    M(0,1) = u(2,0);
    M(2,1) = -u(0,0);
    M(0,2) = -u(1,0);
    M(1,2) = u(0,0);
    
    //Reproject back into space of rotation matrices
    closest_rotation(M,R);

}
