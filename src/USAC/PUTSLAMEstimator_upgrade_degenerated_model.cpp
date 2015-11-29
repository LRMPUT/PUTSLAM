#include "../include/USAC/PUTSLAMEstimator.h"

//// COPIED FROM PUTSLAM:

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

// ============================================================================================
// upgradeDegenerateModel: try to upgrade degenerate model to non-degenerate by sampling from
// the set of outliers to the degenerate model
// ============================================================================================
unsigned int PUTSLAMEstimator::upgradeDegenerateModel()
{
	// MF:
	// simplest solution - no upgrading at all
	// copied from HomogEstimator
	// the FundmatrixEstimator is usign something more complicated

	return 0;
}