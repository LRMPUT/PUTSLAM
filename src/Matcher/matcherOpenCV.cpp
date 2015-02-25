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

putslam::Matcher* putslam::createMatcherOpenCV(
		const std::string _parametersFile) {
	matcher.reset(new MatcherOpenCV(_parametersFile));
	return matcher.get();
}

// MatcherSURF
MatcherOpenCV::MatcherOpenCV(void) :
		Matcher("OpenCV Matcher") {
	initVariables();
}

MatcherOpenCV::MatcherOpenCV(const std::string _parametersFile) :
		Matcher("OpenCVMatcher", _parametersFile) {
	initVariables();
}
;
MatcherOpenCV::MatcherOpenCV(const std::string _name,
		const std::string _parametersFile) :
		Matcher(_name, _parametersFile) {
	initVariables();
}
;

void MatcherOpenCV::initVariables() {
	featureDetector = NULL;
	descriptorExtractor = NULL;

	// TODO: LOAD IT FROM FILE !!!
	cameraMatrixMat = cv::Mat::zeros(3, 3, CV_32FC1);
	distortionCoeffsMat = cv::Mat::zeros(1, 5, CV_32FC1);

//	cameraMatrixMat.at<float>(0,0) = 517.3f;
//	cameraMatrixMat.at<float>(0,2) = 318.6f;
//	cameraMatrixMat.at<float>(1,1) = 516.5f;
//	cameraMatrixMat.at<float>(1,2) = 255.3f;
//	cameraMatrixMat.at<float>(2,2) = 1.0f;
//
//	distortionCoeffsMat.at<float>(0) = -0.0410;
//	distortionCoeffsMat.at<float>(1) = 0.3286;
//	distortionCoeffsMat.at<float>(2) = 0.0087;
//	distortionCoeffsMat.at<float>(3) = 0.0051;
//	distortionCoeffsMat.at<float>(4) = -0.5643;

	cameraMatrixMat.at<float>(0, 0) = 480.6f;
	cameraMatrixMat.at<float>(0, 2) = 319.5f;
	cameraMatrixMat.at<float>(1, 1) = -480.6f;
	cameraMatrixMat.at<float>(1, 2) = 239.5f;
	cameraMatrixMat.at<float>(2, 2) = 1.0f;

	// Initialize detection
	if (matcherParameters.OpenCVParams.detector == "FAST")
		featureDetector.reset(new cv::FastFeatureDetector());
	else if (matcherParameters.OpenCVParams.detector == "ORB")
		featureDetector.reset(new cv::OrbFeatureDetector());
	else if (matcherParameters.OpenCVParams.detector == "SURF")
	{
        //featureDetector.reset(new cv::SurfFeatureDetector());
        featureDetector.reset(
                new cv::DynamicAdaptedFeatureDetector(
                        new cv::SurfAdjuster(3, true), 50, 150));

	}else if (matcherParameters.OpenCVParams.detector == "SIFT")
		featureDetector.reset(new cv::SiftFeatureDetector());
	else
		featureDetector.reset(new cv::SurfFeatureDetector());

	// Initialize description
	if (matcherParameters.OpenCVParams.descriptor == "BRIEF")
		descriptorExtractor.reset(new cv::BriefDescriptorExtractor());
	else if (matcherParameters.OpenCVParams.descriptor == "ORB")
		descriptorExtractor.reset(new cv::OrbDescriptorExtractor());
	else if (matcherParameters.OpenCVParams.descriptor == "SURF")
		descriptorExtractor.reset(new cv::SurfDescriptorExtractor());
	else if (matcherParameters.OpenCVParams.descriptor == "SIFT")
		descriptorExtractor.reset(new cv::SiftDescriptorExtractor());

	// Initialize matcher
	// We are always using the cross-check option to remove false matches
	// We are using L2 norm as descriptors are floating-point type
	if (matcherParameters.OpenCVParams.descriptor == "SURF"
			|| matcherParameters.OpenCVParams.descriptor == "SIFT")
		matcher.reset(new cv::BFMatcher(cv::NORM_L2, true));
	// In other case use Hamming distance as descriptors are binary
	else
		matcher.reset(new cv::BFMatcher(cv::NORM_HAMMING, true));
}

MatcherOpenCV::~MatcherOpenCV(void) {
}

const std::string& MatcherOpenCV::getName() const {
	return name;
}


/// Detect features
std::vector<cv::KeyPoint> MatcherOpenCV::detectFeatures(cv::Mat rgbImage) {

	cv::Mat grayImage;
	cv::cvtColor(rgbImage, grayImage, CV_RGB2GRAY);

	std::vector<cv::KeyPoint> raw_keypoints;
//	featureDetector.get()->detect(grayImage, raw_keypoints);

	int grayImageWidth = grayImage.cols, grayImageHeight = grayImage.rows;
	int stripesCount = 6;
	for (int i = 0; i < stripesCount; i++) {
		std::vector<cv::KeyPoint> keypointsInROI;
		cv::Mat roiBGR(grayImage, cv::Rect(0, i * grayImageHeight/stripesCount, grayImageHeight, grayImageHeight/stripesCount));
		cv::Mat roiD(grayImage, cv::Rect(0, i * grayImageHeight/stripesCount, grayImageHeight, grayImageHeight/stripesCount));

		featureDetector.get()->detect(roiBGR, keypointsInROI);

		if (matcherParameters.verbose>1)
			std::cout<<"MatcherOpenCV: Stripe " << i << " : " << keypointsInROI.size() << " keypoints" << std::endl;

		std::sort(keypointsInROI.begin(), keypointsInROI.end(), MatcherOpenCV::compare_response);
		for (int j = 0; j < keypointsInROI.size() && j < 75; j++) {
			keypointsInROI[j].pt.y += i * grayImageHeight/stripesCount;
			raw_keypoints.push_back(keypointsInROI[j]);
		}
	}




	// It is better to have them sorted according to their response strength
	std::sort(raw_keypoints.begin(), raw_keypoints.end(), MatcherOpenCV::compare_response);

	if ( raw_keypoints.size() > 500)
		raw_keypoints.resize(500);

	return raw_keypoints;
}


/// Describe features
cv::Mat MatcherOpenCV::describeFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> features) {

	cv::Mat descriptors;

	// almost in OpenCV
	if (matcherParameters.OpenCVParams.descriptor == "LDB") {
		//LDB ldb;
		//ldb.compute(x, features, descriptors, false);
	} else
		descriptorExtractor.get()->compute(rgbImage, features, descriptors);

	return descriptors;
}

/// Perform matching
std::vector<cv::DMatch> MatcherOpenCV::performMatching(cv::Mat prevDescriptors,
		cv::Mat descriptors) {

	// We are doing the matching
	std::vector<cv::DMatch> matches;
	matcher.get()->match(prevDescriptors, descriptors, matches);

	return matches;
}
