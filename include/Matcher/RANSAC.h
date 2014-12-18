/** @file RANSAC.h
 *
 * The robust estimation method to find transformation
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
