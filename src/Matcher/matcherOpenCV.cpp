/** @file matcherOpenCV.cpp
 *
 * \brief The matching methods implemented for detectors and descriptors in OpenCV
 * \author Michal Nowicki
 *
 */
#include "../include/Matcher/matcherOpenCV.h"
#include <memory>
#include <stdexcept>

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace putslam;

/// A single instance of OpenCV matcher
MatcherOpenCV::Ptr matcher;

putslam::Matcher* putslam::createMatcherOpenCV(void) {
	matcher.reset(new MatcherOpenCV());
	return matcher.get();
}

putslam::Matcher* putslam::createMatcherOpenCV(const std::string _parametersFile) {
	matcher.reset(new MatcherOpenCV(_parametersFile));
	return matcher.get();
}

// MatcherSURF
MatcherOpenCV::MatcherOpenCV(void) :
		Matcher("OpenCV Matcher") {

}


const std::string& MatcherOpenCV::getName() const {
	return name;
}

/// Detect features
std::vector<cv::KeyPoint> MatcherOpenCV::detectFeatures(cv::Mat rgbImage) {
	cv::FeatureDetector *featureDetector;

	if (matcherParameters.OpenCVParams.detector == "FAST")
		featureDetector = new cv::FastFeatureDetector();
	else if (matcherParameters.OpenCVParams.detector == "ORB")
		featureDetector = new cv::OrbFeatureDetector();
	else if (matcherParameters.OpenCVParams.detector == "SURF")
		featureDetector = new cv::SurfFeatureDetector();
	else if (matcherParameters.OpenCVParams.detector == "SIFT")
		featureDetector = new cv::SiftFeatureDetector();
	else
		featureDetector = new cv::SurfFeatureDetector();

	cv::Mat grayImage;
    cv::cvtColor(rgbImage, grayImage, CV_RGB2GRAY);

	std::vector<cv::KeyPoint> raw_keypoints;
	featureDetector->detect(grayImage, raw_keypoints);
	delete featureDetector;

	return raw_keypoints;
}

/// Describe features
cv::Mat MatcherOpenCV::describeFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> features) {
	cv::DescriptorExtractor * extractor;

	if (matcherParameters.OpenCVParams.descriptor == "BRIEF")
		extractor = new cv::BriefDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "ORB")
		extractor = new cv::OrbDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "SURF")
		extractor = new cv::SurfDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "SIFT")
		extractor = new cv::SiftDescriptorExtractor();

	cv::Mat descriptors;
	extractor->compute(rgbImage, features, descriptors);
	delete extractor;

	return descriptors;
}

/// Perform matching
std::vector<cv::DMatch> MatcherOpenCV::performMatching(cv::Mat prevDescriptors,
		cv::Mat descriptors) {

	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(prevDescriptors, descriptors, matches);
	return matches;
}

/// Reset matching
void MatcherOpenCV::reset() {

}
