#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <unsupported/Eigen/BVH>
#include <igl/per_face_normals.h> 
#include <vector>

struct Triangle 
{
    // each row is one vertex on this triangle
    Eigen::Matrix3d vertices;

    // row in FY that defines this triangle
    int frow;
};

namespace Eigen 
{
    AlignedBox3d bounding_box(Triangle t) 
    {
        Eigen::Vector3d corner1 = t.vertices.colwise().minCoeff();
        Eigen::Vector3d corner2 = t.vertices.colwise().maxCoeff();
        return AlignedBox3d(corner1, corner2);
    }
}

struct PointTriangleMinimizer
{
    typedef double Scalar;
    Eigen::RowVector3d query_point;

    double shortest_dist = -1;
    Eigen::RowVector3d closest_point;

    // the row in FY that defines the triangle on which closest_point lies
    int frow;

    PointTriangleMinimizer(Eigen::RowVector3d x) 
    {
        query_point = x;
    }

    // Shortest distance from query_point to bounding box b
    double minimumOnVolume(const Eigen::AlignedBox3d & b) 
    {
        Eigen::Matrix3d D;
        D << b.corner(b.BottomLeftFloor) - query_point.transpose(),
            Eigen::Vector3d::Zero(),
            query_point.transpose() - b.corner(b.TopRightCeil);
        return D.rowwise().maxCoeff().norm();
    }

    // Shortest distance from query_point to triangle t
    double minimumOnObject(const Triangle & t) 
    {
        double d;
        Eigen::RowVector3d p;
        point_triangle_distance(query_point, t.vertices.row(0), t.vertices.row(1), t.vertices.row(2), d, p);

        if (d < shortest_dist || shortest_dist == -1) 
        {
            shortest_dist = d;
            closest_point = p;
            frow = t.frow;
        }

        return d;
    }
};


void point_mesh_distance(
    const Eigen::MatrixXd & X,
    const Eigen::MatrixXd & VY,
    const Eigen::MatrixXi & FY,
    Eigen::VectorXd & D,
    Eigen::MatrixXd & P,
    Eigen::MatrixXd & N)
{
    Eigen::MatrixXd all_normals;
    igl::per_face_normals(VY, FY, all_normals);

    D.resize(X.rows());
    P.resize(X.rows(), 3);
    N.resize(X.rows(), 3);

    std::vector<Triangle> triangles;
    for (int j = 0; j < FY.rows(); j++) {
        Eigen::RowVector3i fy = FY.row(j);
        Triangle t;
        t.vertices << VY.row(fy[0]), VY.row(fy[1]), VY.row(fy[2]);
        t.frow = j;
        triangles.push_back(t);
    }

    Eigen::KdBVH<double, 3, Triangle> triangle_tree(triangles.begin(), triangles.end());

    for (int i = 0; i < X.rows(); i++) 
    {
        PointTriangleMinimizer minimizer(X.row(i));
        D[i] = Eigen::BVMinimize(triangle_tree, minimizer);
        P.row(i) = minimizer.closest_point;
        N.row(i) = all_normals.row(minimizer.frow);
    }
}
