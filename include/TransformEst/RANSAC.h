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
	enum ERROR_VERSION {EUCLIDEAN_ERROR, REPROJECTION_ERROR, EUCLIDEAN_AND_REPROJECTION_ERROR};
	struct parameters {
		int verbose;
		int errorVersion, errorVersionVO, errorVersionMap;
        double inlierThresholdEuclidean, inlierThresholdReprojection;
		double minimalInlierRatioThreshold;
		int usedPairs;
		int iterationCount;
	};

	RANSAC(RANSAC::parameters RANSACParameters, cv::Mat cameraMatrix = cv::Mat());

	/**
	 * Method used to robustly estimate transformation from given two sets of
	 * 3D features and potentially correct matches between those sets
	 *
	 * prevFeatures 	-- 	3D locations of features from the first set
	 * features 		--	3D locations of features currently observed (2nd set)
	 * matches			--	vector of indices of corresponding features
	 * inlierMatches	--  vector of matches considered as inliers
	 */
	Eigen::Matrix4f estimateTransformation(
			std::vector<Eigen::Vector3f> prevFeatures,
			std::vector<Eigen::Vector3f> features,
			std::vector<cv::DMatch> matches,
			std::vector<cv::DMatch> & bestInlierMatches);

private:
	cv::Mat cameraMatrix;
	parameters RANSACParams;

	enum TransfEstimationType {
		UMEYAMA, G2O
	};

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
	 * usedType					-- 	algorithm used in transformation estimation
	 */
	bool computeTransformationModel(
			const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			Eigen::Matrix4f &transformationModel,
			TransfEstimationType usedType = UMEYAMA);

	/**
	 * Method used to check if the found transformation does not exceed sensible constrains:
	 * transformationModel 		-- transformation to check
	 */
	bool checkModelFeasibility(Eigen::Matrix4f transformationModel);

	/**
	 * Method used to compute the inlierRatio based on 3D Euclidean error:
	 *
	 * prevFeatures				-- 	first set of 3D features
	 * features					--	second set of 3D features
	 * matches					-- 	vector of matches to be determined as inliers or outliers
	 * transformationModel		--	transformation used in evaluation
	 * modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
	 */
	float computeInlierRatioEuclidean(const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			const Eigen::Matrix4f transformationModel,
			std::vector<cv::DMatch> &modelConsistentMatches);

	/**
	 * Method used to compute the inlierRatio based on reprojection error:
	 *
	 * prevFeatures				-- 	first set of 3D features
	 * features					--	second set of 3D features
	 * matches					-- 	vector of matches to be determined as inliers or outliers
	 * transformationModel		--	transformation used in evaluation
	 * modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
	 */
	float computeInlierRatioReprojection(
			const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			const Eigen::Matrix4f transformationModel,
			std::vector<cv::DMatch> &modelConsistentMatches);

	/**
		 * Method used to compute the inlierRatio based on eulidean and reprojection error (simultaneously):
		 *
		 * prevFeatures				-- 	first set of 3D features
		 * features					--	second set of 3D features
		 * matches					-- 	vector of matches to be determined as inliers or outliers
		 * transformationModel		--	transformation used in evaluation
		 * modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
		 */
		float computeInlierRatioEuclideanAndReprojection(
				const std::vector<Eigen::Vector3f> prevFeatures,
				const std::vector<Eigen::Vector3f> features,
				const std::vector<cv::DMatch> matches,
				const Eigen::Matrix4f transformationModel,
				std::vector<cv::DMatch> &modelConsistentMatches);

	/**
	 * Method used to compare two transformation models and save better one
	 *
	 * inlierRatio 				--	inlier ratio of the model
	 * transformationModel 		-- 	transformation as 4x4 matrix
	 * modelConsistentMatches	--	feature matches that can be considered as inliers using provided transformation model
	 * bestInlierRatio 			--	inlier ratio of the best model, place to save better inlierRatio
	 * bestTransformationModel 	-- 	transformation as 4x4 matrix of the best model,
	 * 								place to save better transformation
	 * bestInlierMatches		-- 	feature matches that can be considered as inliers using bestTransformation model
	 */

	inline void saveBetterModel(const float inlierRatio,
			const Eigen::Matrix4f transformationModel,
			std::vector<cv::DMatch> modelConsistentMatches,
			float &bestInlierRatio, Eigen::Matrix4f & bestTransformationModel,
			std::vector<cv::DMatch> &bestInlierMatches);

	/**
	 * Method estimating needed number of iterations for RANSAC given:
	 *
	 * inlierRatio 			-- 	estimated number of inliers (can be computed based on currently best model)
	 * successProbability 	-- 	in most cases 0.95 or 0.98
	 * numberOfPairs 		--	number of pairs used to create a transformation model
	 */
	inline int computeRANSACIteration(double inlierRatio,
			double successProbability = 0.98, int numberOfPairs = 3);

};

#endif // _RANSAC_H_
