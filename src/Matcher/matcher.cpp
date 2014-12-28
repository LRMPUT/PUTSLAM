/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/Matcher/RGBD.h"

using namespace putslam;

void Matcher::loadInitFeatures(const SensorFrame &sensorData)
{
	// Detect salient features
	prevFeatures = detectFeatures(sensorData.image);

	// Show detected features
	//showFeatures(next_frame.image, prevFeatures);

	// Remove features without depth
	RGBD::removeFeaturesWithoutDepth(prevFeatures, sensorData.depth);

	// Describe salient features
	prevDescriptors = describeFeatures(sensorData.image, prevFeatures);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeatures, sensorData.depth);

	// Save rgb/depth images
	prevRgbImage = sensorData.image.clone();
	prevDepthImage = sensorData.depth.clone();
}


bool Matcher::match(const SensorFrame& sensorData, Eigen::Matrix4f &estimatedTransformation) {
	// Detect salient features
	std::vector<cv::KeyPoint> features = detectFeatures(sensorData.image);

	// Remove features without depth
	RGBD::removeFeaturesWithoutDepth(features, sensorData.depth);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(sensorData.image, features);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(features, sensorData.depth);

	// Visualize matches
	//showMatches(prevRgbImage, prevFeatures, next_frame.image, features, matches);

	// RANSAC
	RANSAC ransac(matcherParameters.RANSACParams);
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D, features3D, matches);

	// Save computed values for next iteration
	features.swap(prevFeatures);
	features3D.swap(prevFeatures3D);
	cv::swap(descriptors, prevDescriptors);

	// Save rgb/depth images
	prevRgbImage = sensorData.image.clone();
	prevDepthImage = sensorData.depth.clone();

	return false;
}


void Matcher::showFeatures(cv::Mat rgbImage, std::vector<cv::KeyPoint> features)
{
	cv::Mat imageToShow;
	cv::drawKeypoints(rgbImage,prevFeatures,imageToShow);

	cv::imshow("Showing features", imageToShow);
	cv::waitKey(10000);
}

void Matcher::showMatches(cv::Mat prevRgbImage,
		std::vector<cv::KeyPoint> prevFeatures, cv::Mat rgbImage,
		std::vector<cv::KeyPoint> features, std::vector<cv::DMatch> matches) {

	cv::Mat imageToShow;
	cv::drawMatches(prevRgbImage, prevFeatures, rgbImage, features, matches,
			imageToShow);

	cv::imshow("Showing matches", imageToShow);
	cv::waitKey(10000);
}
