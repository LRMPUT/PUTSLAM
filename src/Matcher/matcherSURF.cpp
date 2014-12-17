#include "../include/Matcher/matcherSURF.h"
#include <memory>
#include <stdexcept>

#include <opencv2/nonfree/nonfree.hpp>

using namespace putslam;

/// A single instance of SURF matcher
MatcherSURF::Ptr matcher;

putslam::Matcher* putslam::createMatcherSURF(void) {
	matcher.reset(new MatcherSURF());
	return matcher.get();
}

// MatcherSURF
MatcherSURF::MatcherSURF(void) :
		Matcher("SURF Matcher") {

}

const std::string& MatcherSURF::getName() const {
	return name;
}

/// Detect features
std::vector<cv::KeyPoint> MatcherSURF::detectFeatures(cv::Mat rgbImage) {
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
cv::Mat MatcherSURF::describeFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> features) {
	cv::DescriptorExtractor * extractor;
	extractor = new cv::SurfDescriptorExtractor();

	cv::Mat descriptors;
	extractor->compute(rgbImage, features, descriptors);
	delete extractor;

	return descriptors;
}

/// Perform matching
std::vector<cv::DMatch> MatcherSURF::performMatching(cv::Mat prevDescriptors,
		cv::Mat descriptors) {

	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(prevDescriptors, descriptors, matches);
	return matches;
}

/// Reset matching
void MatcherSURF::reset() {

}
