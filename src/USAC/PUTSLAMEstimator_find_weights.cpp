#include "../include/USAC/PUTSLAMEstimator.h"

//// COPIED FROM PUTSLAM:

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

//// it's used in local optimization

// ============================================================================================
// findWeights: given model and points, compute weights to be used in local optimization
// ============================================================================================
void PUTSLAMEstimator::findWeights(unsigned int modelIndex, const std::vector<unsigned int>& inliers,
	unsigned int numInliers, double* weights)
{
	// MF:
	// simplest solution - all the points have the same weight
	// copied from HomogEstimator
	// the FundmatrixEstimator is usign something more complicated

	for (unsigned int i = 0; i < numInliers; ++i)
	{
		weights[i] = 1.0;
	}
}

