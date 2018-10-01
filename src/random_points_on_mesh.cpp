#include "random_points_on_mesh.h"
#include <iostream>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  X.resize(n,3);

  // create cumulative area vector
  Eigen::VectorXd areas, cum_areas;
  igl::doublearea(V, F, areas);
  igl::cumsum(areas, 1, cum_areas);
  cum_areas = cum_areas/cum_areas(cum_areas.rows() - 1);

  for(int i = 0;i<X.rows();i++){
    // Sample face
    double z = ((double) rand()) / RAND_MAX;
    
    int last = F.rows();
    int first = 0;
    int face_idx = 0;
    
    // Binary search for face_idx
    while(true){
      if (first == last || first == last - 1){
        face_idx = first;
        break;
      }
      face_idx = (first + last)/2;
      
      if (cum_areas(face_idx) < z){
        first = face_idx;
      }
      else {
        last = face_idx;
      }
      // std::cout << face_idx << "  ," << first << "  ," << last << "  ," << z << "  ," << cum_areas(face_idx) << "  ," << cum_areas(face_idx + 1) << "\n";
    }

    // Get random point on face
    double alpha = ((double) rand()) / RAND_MAX;
    double beta = ((double) rand()) / RAND_MAX;
    if (alpha + beta > 1){
      alpha = 1 - alpha;
      beta = 1 - beta;
    }

    Eigen::VectorXd va = V.row(F(face_idx, 0)), vb = V.row(F(face_idx, 1)), vc = V.row(F(face_idx, 2));
    X.row(i) = va + alpha*(vb - va) + beta*(vc - va);
  }
}