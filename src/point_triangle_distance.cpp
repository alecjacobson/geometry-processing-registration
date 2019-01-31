#include "point_triangle_distance.h"
#include "iostream"
#include <Eigen/Geometry>

Eigen::RowVector3d PxPy(const Eigen::RowVector3d &x, const Eigen::RowVector3d &y) {
    return y - x;
}

Eigen::RowVector3d PxPy_norm(const Eigen::RowVector3d &x, const Eigen::RowVector3d &y) {
    return PxPy(x, y) / PxPy(x, y).norm();
}

bool clockwise(double f) {
    return f < 0;
}

bool anti_clockwise(double f) {
    return f > 0;
}

bool is_outside(const Eigen::RowVector3d &x, const Eigen::RowVector3d &y, const Eigen::RowVector3d &pop,
                const Eigen::RowVector3d &Np) {

    double val = PxPy(pop, x).cross(PxPy(pop, y)).dot(Np);
    return val < 0;
}

void
getPointDistance(const Eigen::RowVector3d &p1, const Eigen::RowVector3d &p2, const Eigen::RowVector3d &po,
                 const Eigen::RowVector3d &pop,
                 const double popop_l, double &d,
                 Eigen::RowVector3d &p) {

    //P1 and P2 are refereed to equation 7.
    Eigen::RowVector3d R = PxPy(pop, p2).cross(PxPy(pop, p1)).cross(PxPy(p1, p2));
    double cos_gamma = PxPy(pop, p1).dot(R) / (PxPy(pop, p1).norm() * R.norm());
    double poppopp_l = PxPy(pop, p1).norm() * cos_gamma;
    Eigen::RowVector3d poppopp = poppopp_l * R / R.norm();
    Eigen::RowVector3d p0pp = pop + poppopp;//eq12
    Eigen::RowVector3d num = p0pp - p1;
    Eigen::RowVector3d denom = p2 - p1;

    double t = num.sum() / (denom.sum() + 1e-6);

    if (t >= 0 && t <= 1) {
        d = sqrt(poppopp_l * poppopp_l + popop_l * popop_l);
        p = p0pp;
    } else if (t < 0) {
        d = PxPy(po, p1).norm();
        p = p1;
    } else if (t > 1) {
        d = PxPy(po, p2).norm();
        p = p2;
    }
}


void point_triangle_distance(
        const Eigen::RowVector3d &x,
        const Eigen::RowVector3d &a,
        const Eigen::RowVector3d &b,
        const Eigen::RowVector3d &c,
        double &d,
        Eigen::RowVector3d &p) {



    // This implementation follows
    // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.104.4264&rep=rep1&type=pdf
    // 3D Distance from a Point to a Triangle

    //This is a changed of variable to match the equations in the paper.
    Eigen::RowVector3d p0 = x;
    Eigen::RowVector3d p1 = a;
    Eigen::RowVector3d p2 = b;
    Eigen::RowVector3d p3 = c;
    ///


    // "let's get the perpendicular vector to the triangle plane"
    Eigen::RowVector3d Np = PxPy(p1, p2).cross(PxPy(p1, p3));
    double cosalpha = PxPy(p1, p0).dot(Np) / (PxPy(p1, p0).norm() * Np.norm());


    // If it were the case that the projection of p0 onto the plane lay within the triangle
    // this would be the length
    double p0p0p_l = PxPy(p0, p1).norm() * cosalpha;
    //--
    Eigen::RowVector3d p0p0p = (-1.0 * p0p0p_l * Np) / Np.norm();
    Eigen::RowVector3d pop = p0 + p0p0p; // EQ5

    //--
    Eigen::RowVector3d v1 = PxPy_norm(p2, p1) + PxPy_norm(p3, p1);
    Eigen::RowVector3d v2 = PxPy_norm(p3, p2) + PxPy_norm(p1, p2);
    Eigen::RowVector3d v3 = PxPy_norm(p1, p3) + PxPy_norm(p2, p3);

    //--
    double f1 = v1.cross(PxPy(p1, pop)).dot(Np);
    double f2 = v2.cross(PxPy(p2, pop)).dot(Np);
    double f3 = v3.cross(PxPy(p3, pop)).dot(Np);

    // f1 > 0 if P0 is anticlockwise of V1
    // f2 > 0 if P0 is anticlockwise of V2
    // f3 > 0 if P0 is anticlockwise of V3


    if (clockwise(f2) && anti_clockwise(f1)) {
        if (is_outside(p1, p2, pop, Np)) {
            getPointDistance(p1, p2, p0, pop, p0p0p_l, d, p);

        } else {
            d = abs(p0p0p_l);
            p = pop;
        }
    } else if (clockwise(f3) && anti_clockwise(f2)) {
        if (is_outside(p2, p3, pop, Np)) {
            getPointDistance(p2, p3, p0, pop, p0p0p_l, d, p);

        } else {
            d = abs(p0p0p_l);
            p = pop;
        }
    } else if (clockwise(f1) && anti_clockwise(f3)) {
        if (is_outside(p3, p1, pop, Np)) {
            getPointDistance(p3, p1, p0, pop, p0p0p_l, d, p);
        } else {
            d = abs(p0p0p_l);
            p = pop;
        }
    } else {
        assert(true);
    }
//    std::cout << "p:" << p << " d: " << d << std::endl;
}
