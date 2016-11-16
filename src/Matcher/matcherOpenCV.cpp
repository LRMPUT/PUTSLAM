/** @file matcherOpenCV.cpp
 *
 * \brief The matching methods implemented for detectors and descriptors in OpenCV
 * \author Michal Nowicki
 *
 */
#include "../../include/putslam/Matcher/matcherOpenCV.h"

#include <memory>
#include <stdexcept>
#include <set>
#include "Defs/opencv.h"
//#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/imgproc.hpp>

using namespace putslam;

/// A single instance of OpenCV matcher
MatcherOpenCV::Ptr matcherClass, loopClosingMatcherClass;

putslam::Matcher* putslam::createMatcherOpenCV(void) {
	matcherClass.reset(new MatcherOpenCV());
	return matcherClass.get();
}

//putslam::Matcher* putslam::createMatcherOpenCV(
//		const std::string _parametersFile) {
//	matcher.reset(new MatcherOpenCV(_parametersFile));
//	return matcher.get();
//}

putslam::Matcher* putslam::createMatcherOpenCV(
		const std::string _parametersFile,
		const std::string _grabberParametersFile) {
	matcherClass.reset(
			new MatcherOpenCV(_parametersFile, _grabberParametersFile));
	return matcherClass.get();
}

putslam::Matcher* putslam::createloopClosingMatcherOpenCV(
		const std::string _parametersFile,
		const std::string _grabberParametersFile) {
	loopClosingMatcherClass.reset(
			new MatcherOpenCV(_parametersFile, _grabberParametersFile));
	return loopClosingMatcherClass.get();
}

// MatcherSURF
MatcherOpenCV::MatcherOpenCV(void) :
		Matcher("OpenCV Matcher") {
	initVariables();
}


MatcherOpenCV::MatcherOpenCV(const std::string _parametersFile,
		const std::string _grabberParametersFile) :
		Matcher("OpenCVMatcher", _parametersFile, _grabberParametersFile) {
	initVariables();
}

void MatcherOpenCV::initVariables() {
	// Initialize detection
	if (matcherParameters.OpenCVParams.detector == "FAST")
		featureDetector = cv::FastFeatureDetector::create();
	else if (matcherParameters.OpenCVParams.detector == "ORB")
		featureDetector = cv::ORB::create();
	else if (matcherParameters.OpenCVParams.detector == "SURF") {
		featureDetector = cv::xfeatures2d::SURF::create();

		//TODO Couldn't find opencv 3.0 version
//		featureDetector.reset(
//				new cv::DynamicAdaptedFeatureDetector(
//						new cv::SurfAdjuster(2, true), 50, 150));

//		 int maxFeatures = 500;
//		 int rows = 5;
//		 int columns = 5;
//		 featureDetector.reset(new cv::GridAdaptedFeatureDetector(new cv::SurfAdjuster(5.0, true), maxFeatures, rows, columns));

	} else if (matcherParameters.OpenCVParams.detector == "SIFT")
		featureDetector = cv::xfeatures2d::SIFT::create();
	else
		featureDetector = cv::xfeatures2d::SURF::create();



	// Initialize description
	if (matcherParameters.OpenCVParams.descriptor == "ORB")
		descriptorExtractor = cv::ORB::create();
	else if (matcherParameters.OpenCVParams.descriptor == "SURF")
		descriptorExtractor = cv::xfeatures2d::SURF::create();
	else if (matcherParameters.OpenCVParams.descriptor == "SIFT")
		descriptorExtractor = cv::xfeatures2d::SIFT::create();


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

	// Convert to gray
	cv::Mat grayImage;
	cv::cvtColor(rgbImage, grayImage, CV_RGB2GRAY);

	std::vector<cv::KeyPoint> raw_keypoints;
	int grayImageWidth = grayImage.cols, grayImageHeight = grayImage.rows;

	int maximalFeaturesInROI =
						matcherParameters.OpenCVParams.maximalTrackedFeatures * 3
								/ (matcherParameters.OpenCVParams.gridCols
										* matcherParameters.OpenCVParams.gridRows);

	// Let's divide image into boxes/rectangles
	for (int k = 0; k < matcherParameters.OpenCVParams.gridCols; k++) {
		for (int i = 0; i < matcherParameters.OpenCVParams.gridRows; i++) {

			std::vector<cv::KeyPoint> keypointsInROI;
			cv::Mat roiBGR(grayImage,
					cv::Rect(
							k * grayImageWidth
									/ matcherParameters.OpenCVParams.gridCols,
							i * grayImageHeight
									/ matcherParameters.OpenCVParams.gridRows,
							grayImageWidth
									/ matcherParameters.OpenCVParams.gridCols,
							grayImageHeight
									/ matcherParameters.OpenCVParams.gridRows));

			featureDetector.get()->detect(roiBGR, keypointsInROI);

			if (matcherParameters.verbose > 1)
				std::cout << "MatcherOpenCV: Grid (" << k << ", " << i << ") : "
						<< keypointsInROI.size() << " keypoints" << std::endl;

			// Sorting keypoints by the response to choose the bests
			std::sort(keypointsInROI.begin(), keypointsInROI.end(),
					MatcherOpenCV::compare_response);

			// Adding to final keypoints
			for (std::vector<cv::KeyPoint>::size_type j = 0; j < keypointsInROI.size() && (int)j < maximalFeaturesInROI; j++) {
				keypointsInROI[j].pt.x += float(k * grayImageWidth
						/ matcherParameters.OpenCVParams.gridCols);
				keypointsInROI[j].pt.y += float(i * grayImageHeight
						/ matcherParameters.OpenCVParams.gridRows);
				raw_keypoints.push_back(keypointsInROI[j]);
			}
		}
	}

	// It is better to have them sorted according to their response strength
	std::sort(raw_keypoints.begin(), raw_keypoints.end(),
			MatcherOpenCV::compare_response);

	if (matcherParameters.verbose > 1)
		std::cout << "MatcherOpenCV: After joining we have "<< raw_keypoints.size() << " keypoints" << std::endl;

	if ((int)raw_keypoints.size() > matcherParameters.OpenCVParams.maximalTrackedFeatures)
		raw_keypoints.resize(matcherParameters.OpenCVParams.maximalTrackedFeatures);

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
		std::vector<cv::Point2f> &features,
		std::vector<cv::KeyPoint>& prevKeyPoints,
		std::vector<cv::KeyPoint>& keyPoints,
        std::vector<double>& prevDetDists,
        std::vector<double>& detDists) {

	// Some needed variables
	std::vector<uchar> status;
	std::vector<float> err;
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
			matcherParameters.OpenCVParams.maxIter,
			matcherParameters.OpenCVParams.eps);

	// Setting OpenCV flags based on our parameters
	int trackingFlags = 0;
	if (matcherParameters.OpenCVParams.useInitialFlow > 0)
		trackingFlags = cv::OPTFLOW_USE_INITIAL_FLOW;
	if (matcherParameters.OpenCVParams.trackingErrorType > 0)
			trackingFlags |= cv::OPTFLOW_LK_GET_MIN_EIGENVALS;

	// Calculating the movement of features
	cv::calcOpticalFlowPyrLK(prevImg, img, prevFeatures, features, status,
					err,
					cv::Size(matcherParameters.OpenCVParams.winSize,
							matcherParameters.OpenCVParams.winSize),
					matcherParameters.OpenCVParams.maxLevels, termcrit,
					trackingFlags,
					matcherParameters.OpenCVParams.trackingMinEigThreshold);

	keyPoints = prevKeyPoints;
	//copy new positions to keyPoints
	for(std::vector<cv::Point2f>::size_type i = 0; i < features.size(); ++i){
		keyPoints[i].pt = features[i];
	}
	detDists = prevDetDists;

	// This parts removes additional features for which we observed an error above preset threshold
	int errSize = (int)err.size();
	for (int i = 0; i < errSize; i++) {
		if (err[i] > matcherParameters.OpenCVParams.trackingErrorThreshold)
			status[i] = 0;
	}

	// Removing features if they are too close to each other - the feature to remove is based on an error from tracking
	std::set<int> featuresToRemove;
	for (std::vector<cv::Point2f>::size_type i = 0; i < features.size(); i++) {
		for (std::vector<cv::Point2f>::size_type j = i + 1; j < features.size(); j++) {
			if (cv::norm(features[i] - features[j]) < matcherParameters.OpenCVParams.minimalReprojDistanceNewTrackingFeatures) {
				if ( err[i] > err[j])
					featuresToRemove.insert((int)i);
				else
					featuresToRemove.insert((int)j);
			}
		}
	}

	// Returning result in matching-compatible format
	int i = 0, j = 0;
	std::vector<cv::DMatch> matches;
	std::vector<cv::Point2f>::iterator itFeatures = features.begin();
	std::vector<cv::KeyPoint>::iterator itKeyPoints = keyPoints.begin();
    std::vector<double>::iterator itDetDists = detDists.begin();
	std::vector<uchar>::iterator it = status.begin();
	for (; it != status.end(); ++it, i++) {

		// Tracking succeed and the feature is not too close to feature with more precise tracking
		if (*it != 0  && featuresToRemove.find(i) == featuresToRemove.end()) {
			matches.push_back(cv::DMatch(i, j, 0));
			j++;
			++itFeatures;
			++itKeyPoints;
			++itDetDists;
		}
		// Tracking failed -- we remove those features
		else {
			itFeatures = features.erase(itFeatures);
			itKeyPoints = keyPoints.erase(itKeyPoints);
			itDetDists = detDists.erase(itDetDists);
		}
	}

	if (matcherParameters.verbose > 0)
		std::cout << "MatcherOpenCV::performTracking -- features tracked "
				<< matches.size() << " ("
				<< (float)matches.size() * 100.0 / (float)prevFeatures.size() << "%)"
				<< std::endl;

	// Return result
	return matches;
}
