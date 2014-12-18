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
	void estimateTransformation(std::vector<Eigen::Vector3f> prevFeatures,
				std::vector<Eigen::Vector3f> features,
				std::vector<cv::DMatch> matches);

private:
	parameters RANSACParams;


	std::vector<cv::DMatch> getRandomMatches(
			const std::vector<cv::DMatch> matches);

	bool computeTransformationModel(
			const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			Eigen::Matrix4f &transformationModel);

	float computeInlierRatio(const std::vector<Eigen::Vector3f> prevFeatures,
			const std::vector<Eigen::Vector3f> features,
			const std::vector<cv::DMatch> matches,
			const Eigen::Matrix4f transformationModel);

	inline void saveBetterModel(const float inlierRatio,
			const Eigen::Matrix4f transformationModel, float &bestInlierRatio,
			Eigen::Matrix4f & bestTransformationModel);

};

#endif // _RANSAC_H_
