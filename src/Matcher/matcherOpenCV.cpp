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
	initVariables();
}

MatcherOpenCV::MatcherOpenCV(const std::string _parametersFile) : Matcher("OpenCVMatcher", _parametersFile)
{
	initVariables();
};
MatcherOpenCV::MatcherOpenCV(const std::string _name, const std::string _parametersFile) : Matcher(_name, _parametersFile) {
	initVariables();
};

void MatcherOpenCV::initVariables() {
	featureDetector = NULL;
	descriptorExtractor = NULL;
}

MatcherOpenCV::~MatcherOpenCV(void)
{
	if (featureDetector != NULL)
		delete featureDetector;
	if (descriptorExtractor != NULL)
		delete descriptorExtractor;
}

const std::string& MatcherOpenCV::getName() const {
	return name;
}

/// Detect features
std::vector<cv::KeyPoint> MatcherOpenCV::detectFeatures(cv::Mat rgbImage) {


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

	cv::Mat descriptors;

	// inside OpenCV
	if (matcherParameters.OpenCVParams.descriptor == "BRIEF")
		descriptorExtractor = new cv::BriefDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "ORB")
		descriptorExtractor = new cv::OrbDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "SURF")
		descriptorExtractor = new cv::SurfDescriptorExtractor();
	else if (matcherParameters.OpenCVParams.descriptor == "SIFT")
		descriptorExtractor = new cv::SiftDescriptorExtractor();

	// almost in OpenCV
	if (matcherParameters.OpenCVParams.descriptor == "LDB")
	{
		//LDB ldb;
		//ldb.compute(x, features, descriptors, false);
	}
	else
		descriptorExtractor->compute(rgbImage, features, descriptors);


	return descriptors;
}

/// Perform matching
std::vector<cv::DMatch> MatcherOpenCV::performMatching(cv::Mat prevDescriptors,
		cv::Mat descriptors) {

	cv::BFMatcher *matcher;
	// We are always using the cross-check option to remove false matches
	// We are using L2 norm as descriptors are floating-point type
	if ( matcherParameters.OpenCVParams.descriptor == "SURF" || matcherParameters.OpenCVParams.descriptor == "SIFT" )
		matcher = new cv::BFMatcher(cv::NORM_L2, true);
	// In other case use Hamming distance as descriptors are binary
	else
		matcher = new cv::BFMatcher(cv::NORM_HAMMING, true);

	// We are doing the matching
	std::vector<cv::DMatch> matches;
	matcher->match(prevDescriptors, descriptors, matches);
	delete matcher;

	return matches;
}
