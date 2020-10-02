#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <time.h> 
#include <algorithm>
#include <iostream>
using namespace std;

int binarySearch(
	const Eigen::VectorXd &CDF, 
	int startIdx,
	int endIdx,
	double searchVal){

	if (startIdx == endIdx)
		return CDF(startIdx) <= searchVal ? startIdx : -1;

	int midIdx = startIdx + (endIdx - startIdx) / 2;
	if (searchVal <= CDF(midIdx))
    return binarySearch(CDF, startIdx, midIdx, searchVal);
  
  int returnIdx = binarySearch(CDF, midIdx+1, endIdx, searchVal);
  return returnIdx == -1 ? midIdx : returnIdx;
}

int searchIdx(
	const Eigen::VectorXd &CDF, 
	int startIdx,
	int endIdx,
	double searchVal){
	int faceIdx = binarySearch(CDF,startIdx,endIdx,searchVal);
	// clamp result
	if (faceIdx < 0)
		faceIdx = 0;
	if (faceIdx > (CDF.size() - 1))
		faceIdx = CDF.size() - 1;
	return faceIdx;
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  // X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

	// =======================
	// RIP
	// =======================
	// My code (pure random sampling)
	// X.resize(n,3);
 //  int nV = V.rows();
 //  int sampleVIdx[n];
	// bool checkDuplicate;
	// srand(time(NULL));

	// for (int ii = 0; ii < n; ii++){
	// 	checkDuplicate = false;
	// 	do{
	// 		sampleVIdx[ii] = rand() % nV; // sample 0 ~ nV-1
	// 		for (int jj = ii - 1; jj > -1; jj--){
	// 			if (sampleVIdx[ii] == sampleVIdx[jj]) {
	// 				checkDuplicate = true;
	// 			}
	// 			else{
	// 				checkDuplicate = false;
	// 			}
	// 		}
	// 	} while (checkDuplicate);
	// 	X(ii,0) = V(sampleVIdx[ii], 0);
	// 	X(ii,1) = V(sampleVIdx[ii], 1);
	// 	X(ii,2) = V(sampleVIdx[ii], 2);
	// }
	// =======================
	// END RIP
	// =======================

	// My Code (area-weighted uniform sample)
	srand(time(NULL));
	X.resize(n,3);

	Eigen::VectorXd FA, CDF;
	igl::doublearea(V,F,FA);
	igl::cumsum(FA, 1, CDF);
	CDF /= CDF.maxCoeff(); // make CDF min(CDF) ~ 1

	double randNum, u, v, w;
	int FIdx;
	Eigen::Vector3d V0, V1, V2;
	for (int ii = 0; ii < X.rows(); ii++){
		randNum = ((double) rand() / (RAND_MAX));
		FIdx = searchIdx(CDF, 0, CDF.size()-1, randNum);

		// barycentric sample per triangle
		u = ((double) rand() / (RAND_MAX));
		v = ((double) rand() / (RAND_MAX));
		w = 1.0 - u - v;
		V0 = V.row(F(FIdx,0));
		V1 = V.row(F(FIdx,1));
		V2 = V.row(F(FIdx,2));
		X.row(ii) = u*V0 + v*V1 + w*V2;
	}
}

