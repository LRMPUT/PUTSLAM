/** @file RANSAC.h
 *
 * \brief The robust estimation method to find transformation based on two sets of 3D features
 * \author Michal Nowicki
 *
 */

#ifndef _RANSAC_H_
#define _RANSAC_H_

#include <iostream>
#include <vector>
#include "opencv/cv.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

class RANSAC {
public:
	struct parameters {
		int iterationCount;
		int usedPairs;
		float inlierThreshold;
	};

	RANSAC();

	/**
	 * Method used to robustly estimate transformation from given two sets of
	 * 3D features and potentially correct matches between those sets
	 *
	 * prevFeatures 	-- 	3D locations of features from the first set
	 * features 		--	3D locations of features currently observed (2nd set)
	 * matches			--	vector of indices of corresponding features
	 */
	Eigen::Matrix4f estimateTransformation(std::vector<Eigen::Vector3f> prevFeatures,
				std::vector<Eigen::Vector3f> features,
				std::vector<cv::DMatch> matches);

private:
	parameters RANSACParams;

	/**
	 * Method used to return randomly sampled parameters.usedPairs matches out of all matches.
	 *
	 * matches					-- 	vector of all matches
	 */
	std::vector<cv::DMatch> getRandomMatches(
			const std::vector<cv::DMatch> matches);

	/**
	 * Method used to compute transformation model based on:
	 * prevFeatures				-- 	first set of 3D features
	 * features					--	second set of 3D features
	 * matches					-- 	vector of matches used in model creation
	 * transformationModel		-- 	computed transformation saved as a 4x4 matrix
	 */
	bool computeTransformationModel(
			const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			Eigen::Matrix4f &transformationModel);

	/**
	 * Method used to compute the inlierRatio based on:
	 *
	 * prevFeatures				-- 	first set of 3D features
	 * features					--	second set of 3D features
	 * matches					-- 	vector of matches to be determined as inliers or outliers
	 * transformationModel		--	transformation used in evaluation
	 */
	float computeInlierRatio(const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			const Eigen::Matrix4f transformationModel);

	/**
	 * Method used to compare two transformation models and save better one
	 *
	 * inlierRatio 				--	inlier ratio of the model
	 * transformationModel 		-- 	transformation as 4x4 matrix
	 * bestInlierRatio 			--	inlier ratio of the best model, place to save better inlierRatio
	 * bestTransformationModel 	-- 	transformation as 4x4 matrix of the best model,
	 * 								place to save better transformation
	 */

	inline void saveBetterModel(const float inlierRatio,
			const Eigen::Matrix4f transformationModel, float &bestInlierRatio,
			Eigen::Matrix4f & bestTransformationModel);

	/**
	 * Method estimating needed number of iterations for RANSAC given:
	 *
	 * inlierRatio 			-- 	estimated number of inliers (can be computed based on currently best model)
	 * successProbability 	-- 	in most cases 0.95 or 0.98
	 * numberOfPairs 		--	number of pairs used to create a transformation model
	 */
	inline int computeRANSACIteration(double inlierRatio, double successProbability = 0.98, int numberOfPairs = 3);

};

#endif // _RANSAC_H_
