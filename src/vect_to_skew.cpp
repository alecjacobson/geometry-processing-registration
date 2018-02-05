#include "vect_to_skew.h"

Eigen::Matrix3d vect_to_skew(const Eigen::VectorXd v)
{
    Eigen::Matrix3d cross_V;
    cross_V << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return cross_V;
}
