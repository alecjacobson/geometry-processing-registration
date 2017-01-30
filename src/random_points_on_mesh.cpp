#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <map>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

  Eigen::VectorXd area;
  igl::doublearea(V,F,area);
  area = area.array().abs() / 2;
  Eigen::VectorXd cums;
  igl::cumsum(area,1,cums);

  cums /= cums(cums.rows() - 1);
  std::map<double,int> gi;
  for(int i = 0; i < cums.rows(); ++i) {
      gi[cums(i)] = i;
  }

  auto getind = [&](double v) {
      auto it = gi.upper_bound(v);
      it--;
      return it->second;

  };

  auto FV = [&](int t, int ind) {
      return V.row(F(t,ind));
  };

  auto r = (Eigen::VectorXd::Random(n).array() + 1) / 2;
    for(int i = 0; i < n; ++i) {
        int t = getind(r(i));
        auto pr = (Eigen::Vector2d::Random().array()+1)/2;
        X.row(i) = (1 - pr(0) - pr(1)) * FV(t,0) + pr(0) * FV(t,1) + pr(1) * FV(t,2);
    }

}

