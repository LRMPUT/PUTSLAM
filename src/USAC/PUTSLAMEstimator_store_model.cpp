#include "../include/USAC/PUTSLAMEstimator.h"


//// COPIED FROM PUTSLAM:
//inline void RANSAC::saveBetterModel(const float inlierRatio,
//	const Eigen::Matrix4f transformationModel,
//	std::vector<cv::DMatch> modelConsistentMatches, float &bestInlierRatio,
//	Eigen::Matrix4f & bestTransformationModel,
//	std::vector<cv::DMatch> &bestInlierMatches) {
//	if (inlierRatio > bestInlierRatio) {
//		// Save better model
//		bestTransformationModel = transformationModel;
//		bestInlierRatio = inlierRatio;
//		bestInlierMatches.swap(modelConsistentMatches);
//
//		// Update iteration count
//		RANSACParams.iterationCount = std::min(computeRANSACIteration(RANSACParams.minimalInlierRatioThreshold), computeRANSACIteration(bestInlierRatio));
//	}
//}

// ============================================================================================
// storeModel: stores current best model
// this function is called  (by USAC) every time a new best model is found
// ============================================================================================
void PUTSLAMEstimator::storeModel(unsigned int modelIndex, unsigned int numInliers)
{
	// Save better model
	if (RANSACParams.verbose > 1)
		std::cout << "RANSAC: saving best model" << std::endl;

	// MF:
	// code belowe is copied from FundmatrixEstimator
	// the HomogEstimator is usign the same thing

	//for (unsigned int i = 0; i < 9; ++i)
	//{
	//	final_model_params_[i] = *(models_denorm_[modelIndex] + i);
	//}

	// save the current model as the best solution so far
	this->bestInlierRatio = this->inlierRatio;
	this->bestTransformationModel = this->transformationModel;

	std::cout << "Best transormation model: " << this->bestTransformationModel << std::endl;

	this->bestInlierMatches.swap(this->modelConsistentMatches);

	// Print achieved result
	if (RANSACParams.verbose > 1)
		std::cout << "RANSAC: best model inlier ratio : "
				<< this->bestInlierRatio * 100.0 << "%" << std::endl;
}
