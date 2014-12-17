/** @file matcher.cpp
 *
 * implementation -
 *
 */
#include "../include/Matcher/matcher.h"
#include "../include/Matcher/RGBD.h"

using namespace putslam;

bool Matcher::match(const SensorFrame& next_frame) {
	// Detect salient features
	std::vector<cv::KeyPoint> features = detectFeatures(next_frame.image);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(next_frame.image, features);

	// Perform matching
	std::vector<cv::KeyPoint> prevFeatures;
	cv::Mat prevDescriptors;
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Associate depth
	std::vector<float> depth;
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(features, depth);

	// RANSAC
	//		Umeyama

	return false;
}
