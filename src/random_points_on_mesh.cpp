#include "random_points_on_mesh.h"
#include <random>
#include <cstdlib>
#include <igl/doublearea.h>
#include <igl/cumsum.h>

// custom binary search function that returns the index of the first number greater than the input double find
int binary_search_first_greater_element(const Eigen::VectorXd & a, const double & find, int min_index, int max_index, 
                                           int ans_index = 0);

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  // X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());


  // setup random number gen
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  X.resize(n,3);

  for (int rand_num_count=0; rand_num_count<n; rand_num_count++) {
    
    // get a random number for sampling a triangle
    double rndnum = dis(gen);

    // get the areas of the triangles
    Eigen::VectorXd A(F.rows());
    igl::doublearea(V, F, A);

    //double totalarea = A.sum();

    // get the areas cumsummed
    Eigen::VectorXd cumsum_A(A.rows());
    igl::cumsum(A, 1, cumsum_A);

    // normalized cumsum
    Eigen::VectorXd cumsum_A_normalized = cumsum_A/cumsum_A(cumsum_A.rows()-1); //cumsum_A/totalarea;

    // get the index of the random triangle that is chosen based on its area
    int random_triangle_index = binary_search_first_greater_element(cumsum_A_normalized, rndnum, 0, cumsum_A_normalized.rows()-1);

    // given the triangle index, the corr 3 indices of the triangle are v1, v2, v3
    Eigen::VectorXd v1 = V.row(F(random_triangle_index, 0));
    Eigen::VectorXd v2 = V.row(F(random_triangle_index, 1));
    Eigen::VectorXd v3 = V.row(F(random_triangle_index, 2));

    // get two new random numbers alpha and beta to match the writeup
    double alpha = dis(gen);
    double beta = dis(gen);

    // preprocess
    if (alpha + beta > 1.0) {
      alpha = 1 - alpha;
      beta = 1 - beta;
    }

    // set the random point sampled
    X.row(rand_num_count) = v1 + alpha * (v2 - v1) + beta * (v3 - v1);
  }


}


// custom binary search function
int binary_search_first_greater_element(const Eigen::VectorXd & a, const double & find, int min_index, int max_index, 
                                           int ans_index) {
    /*  ans_index variable is to carry a possible answer index into the next recursion cycle so that the 
        first greatest number is only picked */

    int sz = max_index - min_index + 1;

    if (sz == 1) {
        if ((a(ans_index) > a(min_index)) && (a(min_index) > find)) {
            return min_index;
        } else {
            return ans_index;
        }
    }

    if (a(min_index + floor(sz/2)) > find) {
        // bookmark the index as the possible answer and move to search more
        ans_index = min_index + floor(sz/2);

        return binary_search_first_greater_element(a, find, min_index, min_index + floor(sz/2) - 1, ans_index);


    } else {
        return binary_search_first_greater_element(a, find, min_index + floor(sz/2), max_index, ans_index);
    }

    return ans_index;
}

