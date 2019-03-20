#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
    X.resize(n,3);
    Eigen::VectorXd DoubleAreas;
    igl::doublearea(V,F,DoubleAreas);
    Eigen::VectorXd areasum;
    igl::cumsum(DoubleAreas/2,1,areasum);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for(int i = 0;i<X.rows();i++){
        double random_area = dis(gen) * areasum(areasum.rows()-1);
        int triangle_to_use = 0;
        for( int i = 0; i < areasum.rows(); i++){
            if(areasum(i) > random_area){
                break;
            } else {
                triangle_to_use = i;
            }
        }
        double alpha = dis(gen);
        double beta = dis(gen);
        if(alpha + beta > 1){
            alpha = 1 - beta;
            beta = 1 - alpha;
        }
        X.row(i) = alpha * (V.row(F(triangle_to_use,1)) - V.row(F(triangle_to_use,0))) + beta * (V.row(F(triangle_to_use,2)) - V.row(F(triangle_to_use,0))) + V.row(F(triangle_to_use,0));
    }
}

