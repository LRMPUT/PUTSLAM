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

//putslam::Matcher* putslam::createMatcherOpenCV(
//		const std::string _parametersFile) {
//	matcher.reset(new MatcherOpenCV(_parametersFile));
//	return matcher.get();
//}

putslam::Matcher* putslam::createMatcherOpenCV(
		const std::string _parametersFile,
		const std::string _grabberParametersFile) {
	matcher.reset(new MatcherOpenCV(_parametersFile, _grabberParametersFile));
	return matcher.get();
}

// MatcherSURF
MatcherOpenCV::MatcherOpenCV(void) :
		Matcher("OpenCV Matcher") {
	initVariables();
}

//MatcherOpenCV::MatcherOpenCV(const std::string _parametersFile) :
//		Matcher("OpenCVMatcher", _parametersFile) {
//	initVariables();
//}

MatcherOpenCV::MatcherOpenCV(const std::string _parametersFile,
		const std::string _grabberParametersFile) :
		Matcher("OpenCVMatcher", _parametersFile, _grabberParametersFile) {
	initVariables();
}

void MatcherOpenCV::initVariables() {
	featureDetector = NULL;
	descriptorExtractor = NULL;

	// Initialize detection
	if (matcherParameters.OpenCVParams.detector == "FAST")
		featureDetector.reset(new cv::FastFeatureDetector());
	else if (matcherParameters.OpenCVParams.detector == "ORB")
		featureDetector.reset(new cv::OrbFeatureDetector());
	else if (matcherParameters.OpenCVParams.detector == "SURF") {
		featureDetector.reset(new cv::SurfFeatureDetector());
		featureDetector.reset(
				new cv::DynamicAdaptedFeatureDetector(
						new cv::SurfAdjuster(2, true), 50, 150));

//		 int maxFeatures = 500;
//		 int rows = 5;
//		 int columns = 5;
//		 featureDetector.reset(new cv::GridAdaptedFeatureDetector(new cv::SurfAdjuster(5.0, true), maxFeatures, rows, columns));

	} else if (matcherParameters.OpenCVParams.detector == "SIFT")
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
	//featureDetector.get()->detect(grayImage, raw_keypoints);

	int grayImageWidth = grayImage.cols, grayImageHeight = grayImage.rows;
	for (int k = 0; k < matcherParameters.OpenCVParams.gridCols; k++) {
		for (int i = 0; i < matcherParameters.OpenCVParams.gridRows; i++) {

			std::vector<cv::KeyPoint> keypointsInROI;
			cv::Mat roiBGR(grayImage,
					cv::Rect(k * grayImageWidth / matcherParameters.OpenCVParams.gridCols,
							i * grayImageHeight / matcherParameters.OpenCVParams.gridRows,
							grayImageWidth / matcherParameters.OpenCVParams.gridCols,
							grayImageHeight / matcherParameters.OpenCVParams.gridRows));

			featureDetector.get()->detect(roiBGR, keypointsInROI);

			if (matcherParameters.verbose > 1)
				std::cout << "MatcherOpenCV: Grid (" << k << ", " << i << ") : "
						<< keypointsInROI.size() << " keypoints" << std::endl;

			std::sort(keypointsInROI.begin(), keypointsInROI.end(),
					MatcherOpenCV::compare_response);
			for (int j = 0; j < keypointsInROI.size() && j < 75; j++) {
				keypointsInROI[j].pt.x += k * grayImageWidth / matcherParameters.OpenCVParams.gridCols;
				keypointsInROI[j].pt.y += i * grayImageHeight / matcherParameters.OpenCVParams.gridRows;
				raw_keypoints.push_back(keypointsInROI[j]);
			}
		}
	}

	// It is better to have them sorted according to their response strength
	std::sort(raw_keypoints.begin(), raw_keypoints.end(),
			MatcherOpenCV::compare_response);

	if (raw_keypoints.size() > 500)
		raw_keypoints.resize(500);

	return raw_keypoints;
}

/// Describe features
cv::Mat MatcherOpenCV::describeFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> &features) {

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

/// Perform tracking
std::vector<cv::DMatch> MatcherOpenCV::performTracking(cv::Mat prevImg,
		cv::Mat img, std::vector<cv::Point2f> &prevFeatures,
		std::vector<cv::Point2f> &features, std::vector<cv::Point2f> &features2) {

	std::vector<uchar> status;
	std::vector<float> err;
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
			matcherParameters.OpenCVParams.maxIter,
			matcherParameters.OpenCVParams.eps);

	int initialFlow = 0;
	if (matcherParameters.OpenCVParams.useInitialFlow > 0)
		initialFlow = cv::OPTFLOW_USE_INITIAL_FLOW;

	// Calculating movement of features
	cv::calcOpticalFlowPyrLK(prevImg, img, prevFeatures, features, status, err,
			cv::Size(matcherParameters.OpenCVParams.winSize,
					matcherParameters.OpenCVParams.winSize),
			matcherParameters.OpenCVParams.maxLevels, termcrit, initialFlow);

	// Returning result in matching-compatible format
	int i = 0, j = 0;
	std::vector<cv::DMatch> matches;

	std::vector<cv::Point2f>::iterator itFeatures = features.begin();//, itFeatures2 = features2.begin();
	std::vector<uchar>::iterator it = status.begin();
	for (; it != status.end(); ++it, i++) {

		// Tracking succeed
		if (*it != 0) {
			matches.push_back(cv::DMatch(i, j, 0));
			j++;
			++itFeatures;
			//++itFeatures2;
		}
		// Tracking failed -- we remove those features
		else {
			itFeatures = features.erase(itFeatures);
			//itFeatures2 = features2.erase(itFeatures2);
		}
	}

	if (matcherParameters.verbose > 0)
		std::cout << "MatcherOpenCV::performTracking -- features tracked "
				<< matches.size() << " ("
				<< matches.size() * 100.0 / prevFeatures.size() << "%)"
				<< std::endl;

	// Return result
	return matches;
}
