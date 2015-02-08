/** @file RANSAC.c
 *
 * \brief The robust estimation method to find transformation based on two sets of 3f features
 * \author Michal Nowicki
 *
 */
#include "../include/TransformEst/RANSAC.h"

RANSAC::RANSAC(RANSAC::parameters _RANSACParameters) {
	srand(time(0));

	RANSACParams.verbose = _RANSACParameters.verbose;
	RANSACParams.usedPairs = _RANSACParameters.usedPairs;
	RANSACParams.inlierThreshold = _RANSACParameters.inlierThreshold;
	RANSACParams.minimalInlierRatioThreshold =
			_RANSACParameters.minimalInlierRatioThreshold;

	RANSACParams.iterationCount = computeRANSACIteration(0.20);

	if (RANSACParams.verbose > 0) {
		std::cout << "RANSACParams.verbose --> " << RANSACParams.verbose
				<< std::endl;
		std::cout << "RANSACParams.usedPairs --> " << RANSACParams.usedPairs
				<< std::endl;
		std::cout << "RANSACParams.inlierThreshold --> "
				<< RANSACParams.inlierThreshold << std::endl;
		std::cout << "RANSACParams.minimalInlierRatioThreshold --> "
				<< RANSACParams.minimalInlierRatioThreshold << std::endl;
	}
}

// TODO: MISSING:
// - test of minimal inlierRatio of bestModel
Eigen::Matrix4f RANSAC::estimateTransformation(
		std::vector<Eigen::Vector3f> prevFeatures,
		std::vector<Eigen::Vector3f> features, std::vector<cv::DMatch> matches,
		std::vector<cv::DMatch> & bestInlierMatches) {

	Eigen::Matrix4f bestTransformationModel = Eigen::Matrix4f::Identity();
	float bestInlierRatio = 0.0;

	for (int i = 0; i < RANSACParams.iterationCount; i++) {
		// Randomly select matches
		if (RANSACParams.verbose > 1)
			std::cout << "RANSAC: randomly sampling ids of matches"
					<< std::endl;
		std::vector<cv::DMatch> randomMatches = getRandomMatches(matches);

		// Compute model based on those matches
		if (RANSACParams.verbose > 1)
			std::cout << "RANSAC: computing model based on matches"
					<< std::endl;
		Eigen::Matrix4f transformationModel;
		bool modelComputation = computeTransformationModel(prevFeatures,
				features, randomMatches, transformationModel);

		// TODO: Check if the model is feasible ?
		bool correctModel = checkModelFeasibility(transformationModel);
		if (correctModel) {
			// Evaluate the model
			if (RANSACParams.verbose > 1)
				std::cout << "RANSAC: evaluating the model" << std::endl;
			std::vector<cv::DMatch> modelConsistentMatches;
			float inlierRatio = computeInlierRatio(prevFeatures, features,
					matches, transformationModel, modelConsistentMatches);

			// Save better model
			if (RANSACParams.verbose > 1)
				std::cout << "RANSAC: saving best model" << std::endl;
			saveBetterModel(inlierRatio, transformationModel,
					modelConsistentMatches, bestInlierRatio,
					bestTransformationModel, bestInlierMatches);

			// Print achieved result
			if (RANSACParams.verbose > 1)
				std::cout << "RANSAC: best model inlier ratio : "
						<< bestInlierRatio * 100.0 << "%" << std::endl;
		}

	}

	// Test the number of inliers
	if (bestInlierRatio < RANSACParams.minimalInlierRatioThreshold) {
		bestTransformationModel = Eigen::Matrix4f::Identity();
	}

	// Test for minimal inlierRatio of bestModel
	if (RANSACParams.verbose > 0) {
		std::cout << "RANSAC best model : inlierRatio = "
				<< bestInlierRatio * 100.0 << "%" << std::endl;
		std::cout << "RANSAC best model : " << std::endl
				<< bestTransformationModel << std::endl;
	}
	return bestTransformationModel;
}

// TODO: MISSING:
// - checking against deadlock
// - check if chosen points are not too close to each other
//
std::vector<cv::DMatch> RANSAC::getRandomMatches(
		const std::vector<cv::DMatch> matches) {
	const int matchesSize = matches.size();

	std::vector<cv::DMatch> chosenMatches;
	std::vector<bool> validIndex(matchesSize, true);

	// Loop until we found enough matches
	while (chosenMatches.size() < RANSACParams.usedPairs) {

		// Randomly sample one match
		int sampledMatchIndex = rand() % matchesSize;

		// Check if the match was not already chosen or is not marked as wrong
		if (validIndex[sampledMatchIndex] == true) {

			// Add sampled match
			chosenMatches.push_back(matches[sampledMatchIndex]);

			// Prevent choosing it again
			validIndex[sampledMatchIndex] = false;
		}
	}

	return chosenMatches;
}

// TODO:
// - how to handle Grisetti version?
bool RANSAC::computeTransformationModel(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		Eigen::Matrix4f &transformationModel) {
	Eigen::MatrixXf prevFeaturesMatrix(RANSACParams.usedPairs, 3),
			featuresMatrix(RANSACParams.usedPairs, 3);

	// Create matrices
	for (int j = 0; j < RANSACParams.usedPairs; j++) {
		cv::DMatch p = matches[j];
		prevFeaturesMatrix.block<1, 3>(j, 0) = prevFeatures[p.queryIdx];
		featuresMatrix.block<1, 3>(j, 0) = features[p.trainIdx];
	}

	// Compute transformation
	transformationModel = Eigen::umeyama(featuresMatrix.transpose(),
			prevFeaturesMatrix.transpose(), false);

	// Check if it failed
	if (std::isnan(transformationModel(0, 0))) {
		transformationModel = Eigen::Matrix4f::Identity();
		return false;
	}
	return true;
}

// TODO: - model feasibility
bool RANSAC::checkModelFeasibility(Eigen::Matrix4f transformationModel) {
	return true;
}

float RANSAC::computeInlierRatio(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> &modelConsistentMatches) {
	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f R = transformationModel.block<3, 3>(0, 0);
	Eigen::Vector3f t = transformationModel.block<3, 1>(0, 3);

	int inlierCount = 0;
	// For all matches
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
			it != matches.end(); ++it) {
		// Estimate location of feature from position one after transformation
		Eigen::Vector3f estimatedNewPosition = R * features[it->trainIdx] + t;

		// Compute residual error and compare it to inlier threshold
		if ((estimatedNewPosition - prevFeatures[it->queryIdx]).norm()
				< RANSACParams.inlierThreshold) {
			inlierCount++;
			modelConsistentMatches.push_back(*it);
		}
	}

	// Percent of correct matches
	return float(inlierCount) / matches.size();
}

inline void RANSAC::saveBetterModel(const float inlierRatio,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> modelConsistentMatches, float &bestInlierRatio,
		Eigen::Matrix4f & bestTransformationModel,
		std::vector<cv::DMatch> &bestInlierMatches) {
	if (inlierRatio > bestInlierRatio) {
		// Save better model
		bestTransformationModel = transformationModel;
		bestInlierRatio = inlierRatio;
		bestInlierMatches.swap(modelConsistentMatches);

		// Update iteration count
		RANSACParams.iterationCount = computeRANSACIteration(bestInlierRatio);
	}
}

inline int RANSAC::computeRANSACIteration(double inlierRatio,
		double successProbability, int numberOfPairs) {
	return (log(1 - successProbability)
			/ log(1 - pow(inlierRatio, numberOfPairs)));
}

