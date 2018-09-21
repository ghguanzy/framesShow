#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;


class LinearSystem
{
public:
	struct LUData {
		Mat mLU;
		vector<int> vPivots;
	};

	LinearSystem();
	~LinearSystem();

	static bool LUDecompose(Mat & mMatrix, LUData & vDecomposition);
	static bool LUBackSub(LUData & vDecomposition, const vector<float> & vRHS, vector<float> & vSolution);

};

