#include "point_to_plane_rigid_matching.h"

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
    int k = X.rows();
    Eigen::MatrixXd A;
    A = Eigen::MatrixXd::Zero(3*k,6);

    //column 0
    A.col(0).segment(k,k) = -X.col(2);
    A.col(0).segment(2*k,k) = X.col(1);

    //column 1
    A.col(1).segment(0,k) = X.col(2);
    A.col(1).segment(2*k,k) = -X.col(0);

    //column 2
    A.col(2).segment(0,k) = -X.col(1);
    A.col(2).segment(k,k) = X.col(0);

    //Ones
    A.col(3).segment(0,k) = Eigen::VectorXd::Ones(k);
    A.col(4).segment(k,k) = Eigen::VectorXd::Ones(k);
    A.col(5).segment(2*k,k) = Eigen::VectorXd::Ones(k);

    Eigen::VectorXd v;
    v = Eigen::VectorXd::Zero(3*k);
    v.segment(0,k) = X.col(0)-P.col(0);
    v.segment(k,k) = X.col(1)-P.col(1);
    v.segment(2*k,k) = X.col(2)-P.col(2);
    
    Eigen::MatrixXd Nmat;
    Nmat = Eigen::MatrixXd::Zero(k,3*k);
    Nmat.block(0,0,k,k) = N.col(0).asDiagonal();
    Nmat.block(0,k,k,k) = N.col(1).asDiagonal();
    Nmat.block(0,2*k,k,k) = N.col(2).asDiagonal();
    
    A = Nmat * A;
    v = Nmat * v;
    
    Eigen::VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * v);

    Eigen::Matrix3d RotPart;
    RotPart <<  0,-u(2),u(1),
                u(2),0,-u(0),
                -u(1),u(0),0;
    Eigen::Matrix3d M;
    M = Eigen::Matrix3d::Identity() - RotPart;
    closest_rotation(M,R);
    t = Eigen::Vector3d(u(3),u(4),u(5));
}
