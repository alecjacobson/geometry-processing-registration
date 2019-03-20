#include "point_mesh_distance.h"

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
    // Replace with your code
    P.resizeLike(X);
    D = Eigen::VectorXd(X.rows());
    N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    Eigen::MatrixXd FaceNormals;
    igl::per_face_normals(VY,FY,Eigen::Vector3d(1,1,1).normalized(),FaceNormals);
    for(int i = 0;i<X.rows();i++){
        double curr_distance;
        Eigen::RowVector3d curr_point;
        double min_distance = -1;
        int face_index = 0;
        Eigen::RowVector3d closest_point(0,0,0);
        for(int j = 0; j < FY.rows(); j++){
            
            point_triangle_distance(X.row(i),
                                    VY.row(FY(j,0)),VY.row(FY(j,1)),VY.row(FY(j,2)),
                                    curr_distance,curr_point);
            if(curr_distance < min_distance || min_distance == -1){
                min_distance = curr_distance;
                face_index = j;
                closest_point = curr_point;
            }
        }
        P.row(i) = closest_point;
        D(i) = min_distance;
        N.row(i) = FaceNormals.row(face_index);
    };
}
