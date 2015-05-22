#include "../include/USAC/PUTSLAMEstimator.h"

//// COPIED FROM PUTSLAM:

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

// ============================================================================================
// testSolutionDegeneracy: check if model is degenerate
// ============================================================================================
void PUTSLAMEstimator::testSolutionDegeneracy(bool* degenerateModel, bool* upgradeModel)
{
	// MF:
	// simplest solution - no degeneration testing
	// copied from HomogEstimator
	// the FundmatrixEstimator is usign something more complicated (test if >=5 points in the sample are on a plane)

	*degenerateModel = false;
	*upgradeModel = false;
}