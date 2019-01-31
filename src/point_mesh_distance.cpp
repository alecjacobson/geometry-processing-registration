#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <iostream>
#include <igl/per_face_normals.h>

void point_mesh_distance(
        const Eigen::MatrixXd &X,
        const Eigen::MatrixXd &VY,
        const Eigen::MatrixXi &FY,
        Eigen::VectorXd &D,
        Eigen::MatrixXd &P,
        Eigen::MatrixXd &N) {

    P.resizeLike(X);
    D.resize(P.rows(), 1);
    N.resize(X.rows(), X.cols());

    //computing the normals
    Eigen::MatrixXd NY;
    igl::per_face_normals(VY, FY, NY);

    for (int i = 0; i < X.rows(); i++) {

        // min_p and min_d for a given point x_i
        Eigen::RowVector3d min_p, min_n;
        double min_d = MAXFLOAT;

        //let's find the minimum for Y for a given x
        for (int j = 0; j < FY.rows(); j++) {
            Eigen::RowVector3d p;
            double d;

            point_triangle_distance(X.row(i),
                                    VY.row(FY(j, 0)), //a
                                    VY.row(FY(j, 1)), //b
                                    VY.row(FY(j, 2)), //c
                                    d, p);

            if (d < min_d) {
                min_p = p;
                min_d = d;
                min_n = NY.row(j);
            }
        }
        ///--
        P.row(i) = min_p;
        D(i) = min_d;
        N.row(i) = min_n;

    }

}
