#include "point_triangle_distance.h"
#include <Eigen/Dense>

// Returns area of a triangle with vertices v1, v2, and v3.
double triangle_area(
    const Eigen::Vector3d & v1,
    const Eigen::Vector3d & v2,
    const Eigen::Vector3d & v3) 
{
    return ((v1 - v2).cross(v1 - v3)).norm() / 2;
}

// Returns true if the projection of x onto the plane
// defined by a triangle with vertices a, b, and c 
// is inside the triangle, false otherwise. Outputs
// the coordinates of the projection proj regardless.
bool projection_in_triangle(
    const Eigen::Vector3d & x,
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b,
    const Eigen::Vector3d & c,
    Eigen::Vector3d & proj) 
{
    Eigen::Vector3d n = (c - a).cross(b - a); // normal to plane
    Eigen::Vector3d u = n / n.norm(); // unit normal
    double d = u.dot(x - a);

    proj = x - d * u;

    return triangle_area(a, b, c) >= 
        triangle_area(proj, b, c) + 
        triangle_area(a, proj, c) + 
        triangle_area(a, b, proj);
}

// Outputs the shortest distance d from x to the line
// segment defined by vectors v1 and v2, and the coordinates
// of the closest point p.
void point_line_distance(
    const Eigen::RowVector3d x,
    const Eigen::RowVector3d v1,
    const Eigen::RowVector3d v2,
    double & d,
    Eigen::RowVector3d & p)
{
    Eigen::RowVector3d u = (v1 - v2) / (v1 - v2).norm();
    p = (v1 - x).dot(u) * u;
    double dist = (p - v1).dot(u);

    if (dist < 0 || dist > (v1 - v2).norm())
    {
        // p is not on the line segment, so we set it to the the nearest vertex
        p = (x - v1).norm() < (x - v2).norm() ? v1 : v2;
    }

    d = (x - p).norm();
}

void point_triangle_distance(
    const Eigen::RowVector3d & x,
    const Eigen::RowVector3d & a,
    const Eigen::RowVector3d & b,
    const Eigen::RowVector3d & c,
    double & d,
    Eigen::RowVector3d & p)
{
    double da = (x - a).norm();
    
    // If x is more than 5 triangle side lengths away 
    // from the triangle, approximate distances will suffice
    // so we choose an arbitrary point.
    if (da > 5 * (a - b).norm())
    {
        d = da;
        p = a;
        return;
    }

    Eigen::Vector3d proj;
    if (projection_in_triangle(x.transpose(), a.transpose(), b.transpose(), c.transpose(), proj)) 
    {
        p = proj.transpose();
        d = (x - p).norm();
    }
    else 
    {
        double dists[3];
        Eigen::RowVector3d points[3];

        point_line_distance(x, a, b, dists[0], points[0]);
        point_line_distance(x, a, c, dists[1], points[1]);
        point_line_distance(x, b, c, dists[2], points[2]);

        d = -1;
        for (int i = 0; i < 3; i++) 
        {
            if (d == -1 || d > dists[i]) 
            {
                d = dists[i];
                p = points[i];
            }
        }
    }
}
