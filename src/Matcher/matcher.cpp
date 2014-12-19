/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/Matcher/RGBD.h"
#include "../include/Matcher/RANSAC.h"

using namespace putslam;

bool Matcher::match(const SensorFrame& next_frame) {
	// Detect salient features
	std::vector<cv::KeyPoint> features = detectFeatures(next_frame.image);

	// Remove features without depth
	RGBD::removeFeaturesWithoutDepth(features, next_frame.depth);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(next_frame.image, features);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Associate depth
	std::vector<float> depth;
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(features, depth);

	// RANSAC
	RANSAC ransac;
	Eigen::Matrix4f bestTransformation = ransac.estimateTransformation(prevFeatures3D,features3D,matches);

	// Save computed values for next iteration
	features.swap(prevFeatures);
	features3D.swap(prevFeatures3D);
	cv::swap(descriptors, prevDescriptors);

	return false;
}
