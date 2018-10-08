#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <float.h>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());

  Eigen::MatrixXd allNormals;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), allNormals);

  Eigen::RowVector3d a, b, c, closestP, currentP;
  double minD, currentD;
  int closestFaceInd = 0;

  // loop over all points, add to D, P, and N for each point
  for (int i = 0; i < X.rows(); i++) {

    minD = std::numeric_limits<double>::max();

    // loop over all triangles to find the nearest one
    for (int j = 0; j < FY.rows(); j++) {

      a = VY.row(FY(j, 0));
      b = VY.row(FY(j, 1));
      c = VY.row(FY(j, 2));

      point_triangle_distance(X.row(i), a, b, c, currentD, currentP);

      if (currentD < minD) {
        minD = currentD;
        closestP = currentP;
        closestFaceInd = j;
      }
    }

    D(i) = minD;
    P.row(i) = closestP;
    N.row(i) = allNormals.row(closestFaceInd);
  }
}
