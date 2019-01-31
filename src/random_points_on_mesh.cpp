#include "random_points_on_mesh.h"
#include <iostream>
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <stdlib.h>
#include <random>

int pick_triangle(Eigen::MatrixXd &M, double gamma) {
    //This can be done based on binary search, or std::upper_bound,
    //I am leaving like this for now.
    for (int i = 0; i < M.rows(); i++)
        if (M(i, 0) > gamma) {
            return i;
        }
    return -1;
}

Eigen::VectorXd
pick_inside_triangle(Eigen::VectorXd &v1, Eigen::VectorXd &v2, Eigen::VectorXd &v3, double alpha, double beta) {

    Eigen::VectorXd x;

    if ((alpha + beta) > 1.0) {
        alpha = 1. - alpha;
        beta = 1. - beta;
    }

    x = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
//    std::cout << x << std::endl;
    return x;

}

void random_points_on_mesh(
        const int n,
        const Eigen::MatrixXd &V,
        const Eigen::MatrixXi &F,
        Eigen::MatrixXd &X) {

    Eigen::MatrixXd area, Ajs;


    X.resize(n, 3);

    igl::doublearea(V, F, area);

    //according to documentation double-area computes the area twice.so...
    area = area / 2.0;

    //total area
    double Ax = area.sum();

    // Ajs=(Ai,Ai+Ai+1,Ai+Ai+1+Ai+2,....)
    igl::cumsum(area, 1, Ajs);

    //norm cum sum=Cs
    Eigen::MatrixXd C = Ajs / Ax;


    //source http://www.cplusplus.com/reference/random/uniform_real_distribution/
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    for (int i = 0; i < X.rows(); i++) {
        double gamma = distribution(generator);
        int tid = pick_triangle(C, gamma);
        assert(tid >= 0);

        Eigen::VectorXd v1 = V.row(F(tid, 0));
        Eigen::VectorXd v2 = V.row(F(tid, 1));
        Eigen::VectorXd v3 = V.row(F(tid, 2));


        X.row(i) = pick_inside_triangle(v1, v2, v3, distribution(generator), distribution(generator));

//        std::cout << X.row(i) << std::endl;


    }
}

