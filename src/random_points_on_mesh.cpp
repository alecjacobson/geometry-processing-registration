#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  //for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

  Eigen::VectorXd area;
  igl::doublearea(V,F,area);
  area = area.array().abs() / 2;
  Eigen::VectorXd cums;
  igl::cumsum(area,1,cums);

  cums /= cums(cums.rows() - 1);

  auto getind = [&](double v) {
      auto it = std::upper_bound(cums.data(),cums.data()+cums.rows(),v);
      int d = std::distance(cums.data(),it);
      if(d >= cums.rows()) {
          d = cums.rows() - 1;
      }
      return d;

  };

  auto FV = [&](int t, int ind) {
      return V.row(F(t,ind));
  };

  Eigen::VectorXd r = (Eigen::VectorXd::Random(n).array() + 1) / 2;
  r(0) = 0;
  r(1) = -1;
  r(2) = 1;
  r(3) = 2;
  r(4) = cums(0) - 1e-5;
  r(4) = cums(0) + 1e-5;

    for(int i = 0; i < n; ++i) {
        int t = getind(r(i));
        auto pr = (Eigen::Vector2d::Random().array()+1)/2;
        X.row(i) = (1 - pr(0) - pr(1)) * FV(t,0) + pr(0) * FV(t,1) + pr(1) * FV(t,2);
    }

}

