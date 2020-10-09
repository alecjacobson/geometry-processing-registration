#include "closest_rotation.h"
#include <igl/polar_svd3x3.h>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d omega;
  Eigen::Matrix3d UVt = U * V.transpose();
  omega << 1,0,0,
		   0,1,0,
           0,0, UVt.determinant();


  R = U * omega * V.transpose();
  //igl::polar_svd3x3(M, R);
}
