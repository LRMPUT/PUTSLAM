#include "../include/USAC/PUTSLAMEstimator.h"

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

//// COPIED FROM PUTSLAM:
/**
* Method used to check if the found transformation does not exceed sensible constrains:
* transformationModel 		-- transformation to check
*/
// TODO: - model feasibility
bool PUTSLAMEstimator::checkModelFeasibility(Eigen::Matrix4f transformationModel) {
	return true;
}

// ============================================================================================
// validateModel: check if model computed from minimal sample is valid
// checks oriented constraints to determine model validity
// ============================================================================================
bool PUTSLAMEstimator::validateModel(unsigned int modelIndex)
{
	// MF:
	// simplest solution - no model validation
	// copied from HomogEstimator
	// the FundmatrixEstimator is usign something more complicated

	return this->checkModelFeasibility(this->transformationModel);
}