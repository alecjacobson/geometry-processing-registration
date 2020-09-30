#include "point_triangle_distance.h"

#include <Eigen/Geometry> 

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
    // Reference 
    //  http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.479.8237&rep=rep1&type=pdf


    // Find point `q` on plane defined by a point `p` and normal `n` {z: (z-p) * n = 0}
    //      such that `q` is the (orthogonally) projected point of `x`
    auto project = [](
        const Eigen::RowVector3d& p,
        const Eigen::RowVector3d& n,
        const Eigen::RowVector3d& x) {
            return x - ((x - p).dot(n) / (n.squaredNorm())) * n;
        };

    auto n = (b-a).cross(c-a);
    p = project(a, n, x);

    // Find line connecting center of triangle and the vertices
    auto va = (a-c).normalized() + (a-b).normalized();
    auto vb = (b-c).normalized() + (b-a).normalized();
    auto vc = (c-b).normalized() + (c-a).normalized();

    // Find location of projected point `p`
    //  In particular, `p` is closest to edge represented by `uv`
    auto fa = (va.cross(p-a)).dot(n);
    auto fb = (vb.cross(p-b)).dot(n);
    auto fc = (vc.cross(p-c)).dot(n);

    int ui, vi;
    if (fa > 0 && fb < 0) {
        ui = 0; vi = 1;
    } else if (fb > 0 && fc < 0) {
        ui = 1; vi = 2;
    } else if (fc > 0 && fa < 0) {
        ui = 2; vi = 0;
    }

    auto get = [&](int i){ return (i == 0) ? a : ((i == 1) ? b : c); };
    auto u = get(ui);
    auto v = get(vi);

    if ((v-p).cross(u-p).dot(n) > 0) {
        // project `p` to the closest edge `uv` if `p` outside triangle
        //      the closest point is either a projected point `q` on edge `uv`
        //      or the vertices `u` or `v`
        auto n2 = ((v-p).cross(u-p)).cross(u-v);
        auto q = project(u, n2, p);
        auto t = (q-u).dot(v-u) / (v-u).dot(v-u);

        if (t >= 0 && t <= 1) {
            d = sqrt((p-q).squaredNorm() + (x-p).squaredNorm());
            p = q;
        } else if (t < 0) {
            d = sqrt((p-u).squaredNorm() + (x-p).squaredNorm());
            p = u;
        } else {
            d = sqrt((p-v).squaredNorm() + (x-p).squaredNorm());
            p = v;
        }
    } else {
        // `p` is the closest point on the triangle if `p` inside triangle
        d = (x-p).norm();
    }

}
