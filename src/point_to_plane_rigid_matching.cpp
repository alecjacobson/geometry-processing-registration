#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
#include "closest_rotation.h"
#include <Eigen/Sparse>
#include <Eigen/LU>
using namespace Eigen;

#include <iostream>
using namespace std;

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
    
    typedef Eigen::Triplet<double> T;
    std::vector<T> tripletList;
    tripletList.reserve(X.rows()*3);
    
    
    //cout << "X: " << X << "\n";
    //cout << "P: " << P << "\n";
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
    
    //cout << A << "\n";
    //Compute C for which CX = B;
    //Create sparse matrix?
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
    //cout << "B: " << B_new << "\n";
    //cout << "B: " << B - mat * B1 << "\n";
    //cout << "C: " << C - C1 << "\n";
    //cout << "C1: " << C1 << "\n";
    Eigen::MatrixXd u;
    //u.resize(6,1);
    //cout << "Rows of B: " << B_new.rows() << " Cols of B: " << B_new.cols() << "\n";
    //cout << "Rows of C1: " << C1.rows() << " Cols of C1: " << C1.cols() << "\n";
    u = C1.jacobiSvd(ComputeThinU | ComputeThinV).solve(-B_new);
    

    /*Eigen::MatrixXd Atest, Btest, v;
    Atest = Eigen::MatrixXd::Random(3,3);
    Btest = Eigen::MatrixXd::Random(3,1);
    v = Atest.jacobiSvd(ComputeFullU | ComputeFullV).solve(Btest);
    cout << "Btest: " << Btest << "\n";
    cout << "Result: " << Atest * v << "\n";*/
    M = Eigen::Matrix3d::Identity();
    //cout << "U: " << u << "\n";
    t(0) = u(3,0);
    t(1) = u(4,0);
    t(2) = u(5,0);
    cout << "T: " << t << "\n";
    cout << sqrt((C1 * u + B_new).array().square().mean()) << "\n";
    M(1,0) = -u(2,0);
    M(2,0) = u(1,0);
    M(0,1) = u(2,0);
    M(2,1) = -u(0,0);
    M(0,2) = -u(1,0);
    M(1,2) = u(0,0);
    cout << "M: " << M << "\n";
    
    closest_rotation(M,R);
    cout << "R: " << R << "\n";
    //exit(1);
}
