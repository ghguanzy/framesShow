#include "stdafx.h"
#include "LinearSystem.h"


LinearSystem::LinearSystem()
{
}


LinearSystem::~LinearSystem()
{
}

bool LinearSystem::LUDecompose(Mat & mMatrix, LUData & vDecomposition)
{
	int nRows = mMatrix.rows;
	if (nRows != mMatrix.cols)
		return false;

	vDecomposition.vPivots.resize(nRows);
	vDecomposition.mLU = Mat(nRows, nRows, CV_32FC1);
	std::vector<int> vPivots = vDecomposition.vPivots;
	Mat mLUMatrix = vDecomposition.mLU;

	mLUMatrix = mMatrix;

	float dRowSwaps = 1;

	// scaling of each row
	std::vector<float> vScale(nRows, (float)0);

	float dTemp;
	for (int i = 0; i < nRows; ++i) {			// find scaling for each row
		float dLargest = (float)0;
		for (int j = 0; j < nRows; ++j) {
			if ((dTemp = (float)abs(mLUMatrix.at<float>(i,j))) > dLargest)
				dLargest = dTemp;
		}
		if (dLargest == 0)
			return false;
		vScale[i] = (float)1.0 / dLargest;
	}

	int niMax = 0;
	for (int j = 0; j < nRows; ++j) {		// loop over columns (Crout's method)

											// not entirely sure 
		for (int i = 0; i < j; ++i) {
			float dSum = mLUMatrix.at<float>(i,j);
			for (int k = 0; k < i; ++k)
				dSum -= mLUMatrix.at<float>(i,k) * mLUMatrix.at<float>(k,j);
			mLUMatrix.at<float>(i,j) = dSum;
		}

		// find largest pivot element
		float dLargestPivot = (float)0;
		for (int i = j; i < nRows; ++i) {
			float dSum = mLUMatrix.at<float>(i,j);
			for (int k = 0; k < j; ++k)
				dSum -= mLUMatrix.at<float>(i,k) * mLUMatrix.at<float>(k,j);
			mLUMatrix.at<float>(i,j) = dSum;
			if ((dTemp = vScale[i] * (float)fabs(dSum)) > dLargestPivot) {
				dLargestPivot = dTemp;
				niMax = i;
			}
		}

		// swap rows if pivot is in another column
		if (j != niMax) {
			for (int k = 0; k < nRows; ++k) {
				float dSwap = mLUMatrix.at<float>(niMax,k);
				mLUMatrix.at<float>(niMax,k) = mLUMatrix.at<float>(j,k);
				mLUMatrix.at<float>(j,k) = dSwap;
			}
			dRowSwaps = -dRowSwaps;
			vScale[niMax] = vScale[j];
		}

		vPivots[j] = niMax;
		if (mLUMatrix.at<float>(j,j) == 0)
			mLUMatrix.at<float>(j,j) = FLT_EPSILON;

		if (j != nRows - 1) {
			float dScale = (float)1.0 / mLUMatrix.at<float>(j,j);
			for (int i = j + 1; i < nRows; ++i)
				mLUMatrix.at<float>(i,j) *= dScale;
		}
	}

	return true;
}

bool LinearSystem::LUBackSub(LUData & vDecomposition,
	const vector<float> & vRHS,
	vector<float> & vSolution)
{
	vector<int> & vPivots = vDecomposition.vPivots;
	Mat & mLUMatrix = vDecomposition.mLU;
	int nRows = mLUMatrix.rows;
	if (mLUMatrix.rows != nRows || vPivots.size() != nRows)
		return false;

	// probably there are more efficient ways to do this...
	vSolution = vRHS;

	int nNonVanish = std::numeric_limits<int>::max();
	for (int i = 0; i < nRows; ++i) {
		int nPivot = vPivots[i];
		float dSum = vSolution[nPivot];
		vSolution[nPivot] = vSolution[i];
		if (nNonVanish != std::numeric_limits<int>::max()) {
			for (int j = nNonVanish; j <= i - 1; ++j)
				dSum -= mLUMatrix.at<float>(i,j) * vSolution[j];
		}
		else if (dSum)
			nNonVanish = i;
		vSolution[i] = dSum;
	}
	for (int i = nRows - 1; i >= 0; --i) {
		float dSum = vSolution[i];
		for (int j = i + 1; j < nRows; ++j)
			dSum -= mLUMatrix.at<float>(i,j) * vSolution[j];
		vSolution[i] = dSum / mLUMatrix.at<float>(i,i);
	}

	return true;
}
