#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <cmath>

/* 
First project the given point on the plane of the triangle.
If the projected point lies inside the triangle, that that is the closest point.
If it lies outside, find the 2D projection of the projected point over one of the triangle edges based on
which region it falls into.

If it lies outside, project the prev plane-projected point to all the three sides of the triangle. 
For the three points that are projected, eliminate points that lie outside the triangle by the same check as before.
Now we have one point on some edge. Find the distance, and compare with the distances to the remaining vertices.
Report the least distance.
*/

// This function checks if a given point x_0 *in the plane of the triangle*, exists inside or not.
bool check_if_on_triangle(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c, 
                          const Eigen::RowVector3d & x_0);

// Projecting a plane-projected point over a line given by two coordinate points a and b
void project_on_line(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & x_0, Eigen::RowVector3d & x_0_proj);

// Given a point on the edge and the vertices of the triangle, this function populates the nearest point and distance 
// for the cases where the plane-projected point lies outside of the triangle
void compare_distance_edgepoint_vertices (const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c,
                                          const Eigen::RowVector3d & x_0_ab, const Eigen::RowVector3d & x, 
                                          Eigen::RowVector3d & p, double & d);

// populates minp to be one of p1 or p2 based on which is closest to x
void return_minimum_distant_point_to_x (const Eigen::RowVector3d & p1, const Eigen::RowVector3d & p2, 
                                        const Eigen::RowVector3d & x, Eigen::RowVector3d & minp);

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  // d = 0;
  // p = a;

  Eigen::RowVector3d unit_normal = (b-a).cross(c-b).normalized();

  // compute the difference vector of X with any one of the points on the triangle
  Eigen::RowVector3d diff_vector = x - c;

  // Find the dot product with the unit normal 
  double proj_dot_product = unit_normal.dot(diff_vector);

  // Find the projection point x_0 of the input point x on the plan spanned by the vertices of the triangle
  Eigen::RowVector3d x_0 = x - proj_dot_product * unit_normal;

  // If x_0 lies within the triangle, then abs(proj_dot_product) is the closest distance and x_0 is the
  // closest point

  if (check_if_on_triangle(a, b, c, x_0)) {
    // the projected point is within the triangle!
    p(0) = x_0(0); p(1) = x_0(1); p(2) = x_0(2);
    d = abs(proj_dot_product);
    return;

  } else {
    // the projected point is outside the triangular region
    // in this case the closest point will be either a point on the edge or one of the vertex points

    // first find all the projections of x_0 on each of the edge, and then check if those lie on the triangle or not
    Eigen::RowVector3d x_0_ab, x_0_bc, x_0_ca;
    project_on_line(a, b, x_0, x_0_ab);
    project_on_line(b, c, x_0, x_0_bc);
    project_on_line(c, a, x_0, x_0_ca);

    // check if the points projected on the edge exist inside the triangle. if yes, consider them.
    // then finally, compare dstance against vertices to pick the closest

    Eigen::RowVector3d p_closest_tmp;
    p_closest_tmp(0) = a(0); p_closest_tmp(1) = a(1); p_closest_tmp(2) = a(2); 

    if (check_if_on_triangle(a, b, c, x_0_ab)) {
      compare_distance_edgepoint_vertices(p_closest_tmp, b, c, x_0_ab, x, p_closest_tmp, d);
      
    } 
    
    if (check_if_on_triangle(a, b, c, x_0_bc)) {
      compare_distance_edgepoint_vertices(p_closest_tmp, b, c, x_0_bc, x, p_closest_tmp, d);
      
    } 
    
    if (check_if_on_triangle(a, b, c, x_0_ca)) {
      compare_distance_edgepoint_vertices(p_closest_tmp, b, c, x_0_ca, x, p_closest_tmp, d);
      
    } 
    
    // also check the vertices finally
    compare_distance_edgepoint_vertices(p_closest_tmp, b, c, a, x, p, d);
    return;
     

  }


}

// This function checks if a given point x_0 *in the plane of the triangle*, exists inside or not.
bool check_if_on_triangle(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c, 
                          const Eigen::RowVector3d & x_0) {

  // Check if x_0 is within the triangle region
  
  // Any region within the triangle can be expressed as 
  // b + alpha * (a-b) + beta * (c-b)

  // Decompose the projected point x_0 based on alpha and beta by solving two linear equations obtained by
  // (x_0 - b) dot (a-b), and
  // (x_0 - b) dot (c-b) 

  // Equations worked on the board to solve for alpha and beta as follows:

  double beta_numerator = ((a-b).dot(c-b) * (x_0-b).dot(a-b)) - ((x_0-b).dot(c-b) * (a-b).dot(a-b));
  double beta_denominator = std::pow((c-b).dot(a-b), 2.0) - ((a-b).dot(a-b) * (c-b).dot(c-b));

  double beta = beta_numerator/beta_denominator;

  double alpha = ((x_0-b).dot(a-b) - beta * (c-b).dot(a-b))/(a-b).dot(a-b);

  if ((beta >= 0.0) && (alpha >= 0.0) && (alpha + beta <= 1.0)) {
    // the projected point is within the triangle!
    return true;
  } 
  
  return false;
}

void project_on_line(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & x_0, 
                     Eigen::RowVector3d & x_0_proj) {

  // find the normal dir that points from the line to the point OUTWARDS
  // normal is parallel to AB X (AP X AB) - Note that cross product is not associative! Where P = P(x_0)

  Eigen::RowVector3d AP_cross_AB = (x_0 - a).cross(b - a);

  Eigen::RowVector3d unitnorm_direction = (b-a).cross(AP_cross_AB).normalized();

  x_0_proj = x_0 - ((x_0 - a).dot(unitnorm_direction)) * unitnorm_direction;

}

void compare_distance_edgepoint_vertices (const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c,
                                          const Eigen::RowVector3d & x_0_ab, const Eigen::RowVector3d & x, 
                                          Eigen::RowVector3d & p, double & d) {


Eigen::RowVector3d p_tmp1, p_tmp2, p_tmp3;

return_minimum_distant_point_to_x(a, b, x, p_tmp1);
return_minimum_distant_point_to_x(p_tmp1, c, x, p_tmp2);
return_minimum_distant_point_to_x(p_tmp2, x_0_ab, x, p_tmp3);

p(0) = p_tmp3(0); p(1) = p_tmp3(1); p(2) = p_tmp3(2);
d = (x - p).norm();

}


void return_minimum_distant_point_to_x (const Eigen::RowVector3d & p1, const Eigen::RowVector3d & p2, 
                                        const Eigen::RowVector3d & x, Eigen::RowVector3d & minp) {

if ((p1-x).dot(p1-x) >= (p2-x).dot(p2-x)) {
  minp(0) = p2(0); minp(1) = p2(1); minp(2) = p2(2); 
} else {
  minp(0) = p1(0); minp(1) = p1(1); minp(2) = p1(2);
}

}
