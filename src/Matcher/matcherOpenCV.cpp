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

using namespace putslam;

/// A single instance of OpenCV matcher
MatcherOpenCV::Ptr matcher;

putslam::Matcher* putslam::createMatcherOpenCV(void) {
	matcher.reset(new MatcherOpenCV());
	return matcher.get();
}

// MatcherSURF
MatcherOpenCV::MatcherOpenCV(void) :
		Matcher("OpenCV Matcher") {

}

MatcherOpenCV::MatcherOpenCV(type detector, type descriptor) : Matcher("OpenCV Matcher")
{

}

const std::string& MatcherOpenCV::getName() const {
	return name;
}

/// Detect features
std::vector<cv::KeyPoint> MatcherOpenCV::detectFeatures(cv::Mat rgbImage) {
	cv::FeatureDetector *featureDetector;
	featureDetector = new cv::SurfFeatureDetector();

	cv::Mat grayImage;
	cvtColor(rgbImage, grayImage, CV_RGB2GRAY);

	std::vector<cv::KeyPoint> raw_keypoints;
	featureDetector->detect(grayImage, raw_keypoints);
	delete featureDetector;

	return raw_keypoints;
}

/// Describe features
cv::Mat MatcherOpenCV::describeFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> features) {
	cv::DescriptorExtractor * extractor;
	extractor = new cv::SurfDescriptorExtractor();

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
