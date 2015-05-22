#include "../include/USAC/PUTSLAMEstimator.h"

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

// copied from RANSAC::getRandomMatches from PUTSLAM:
// TODO:
// - checking against deadlock
// - check if chosen points are not too close to each other
//

// ============================================================================================
// validateSample: check if minimal sample is valid
// here, just returns true
// ============================================================================================
bool PUTSLAMEstimator::validateSample()
{
	// MF:
	// simplest solution - no sample validation
	// copied from FundmatrixEstimator
	// the HomogEstimator is usign something more complicated

	return true;
}