/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/Matcher/dbscan.h"

#include <chrono>
#include <assert.h>

using namespace putslam;

// Initial feature detection
void Matcher::detectInitFeatures(const SensorFrame &sensorData) {
	// Detect salient features
	prevKeyPoints = detectFeatures(sensorData.rgbImage);

	if (matcherParameters.verbose > 2)
		std::cout << "Before dbScan: " << prevKeyPoints.size() << std::endl;

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
	dbscan.run(prevKeyPoints);

	if (matcherParameters.verbose > 2)
			std::cout << "After dbScan: " << prevKeyPoints.size() << std::endl;

	// Show detected features
	if (matcherParameters.verbose > 2)
		showFeatures(sensorData.rgbImage, prevKeyPoints);

	// Describe salient features if needed
	prevDescriptors = describeFeatures(sensorData.rgbImage, prevKeyPoints);

	// Extract 2D points from keypoints
	cv::KeyPoint::convert(prevKeyPoints,prevFeaturesDistorted);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevKeyPoints,
			matcherParameters.cameraMatrixMat,
			matcherParameters.distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted,
			sensorData.depthImage, matcherParameters.cameraMatrixMat,
			sensorData.depthImageScale);

	prevDetDists.clear();
	for(std::vector<Eigen::Vector3f>::size_type i = 0; i < prevFeatures3D.size(); ++i){
        double dist = std::sqrt(prevFeatures3D[i][0]*prevFeatures3D[i][0] +
									prevFeatures3D[i][1]*prevFeatures3D[i][1] +
									prevFeatures3D[i][2]*prevFeatures3D[i][2]);

		prevDetDists.push_back(dist);
	}

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;
	prevDepthImageScale = sensorData.depthImageScale;
}

// Running VO
double Matcher::runVO(const SensorFrame& currentSensorFrame,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Matching
	if (matcherParameters.VOVersion
			== Matcher::MatcherParameters::VO_MATCHING) {
		return match(currentSensorFrame, estimatedTransformation, inlierMatches);
		// Tracking
	} else if (matcherParameters.VOVersion
			== Matcher::MatcherParameters::VO_TRACKING) {

//		auto start = std::chrono::high_resolution_clock::now();
		double x = trackKLT(currentSensorFrame, estimatedTransformation,
				inlierMatches);
//		auto end = std::chrono::high_resolution_clock::now();
//		std::cout << "trackKLT = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"<< std::endl;

		return x;

		// Something unrecognized
	} else {
		std::cout
				<< "Unrecognized VO choice -> double check matcherOpenCVParameters.xml"
				<< std::endl;
		return 0.0;
	}
}

// Helper method for trackKLT
void Matcher::mergeTrackedFeatures(
		std::vector<cv::Point2f>& undistortedFeatures2D,
		const std::vector<cv::Point2f>& featuresSandBoxUndistorted,
		std::vector<cv::Point2f>& distortedFeatures2D,
		const std::vector<cv::Point2f>& featuresSandBoxDistorted,
		std::vector<Eigen::Vector3f>& features3D,
		const std::vector<Eigen::Vector3f>& features3DSandBox,
		std::vector<cv::KeyPoint>& keyPoints,
		const std::vector<cv::KeyPoint>& keyPointsSandBox,
        std::vector<double>& detDists,
        const std::vector<double>& detDistsSandBox)
{

	// Merging features - rejecting feature too close to existing ones
	for (std::vector<cv::Point2f>::size_type i = 0; i < featuresSandBoxUndistorted.size(); i++) {
		bool addFeature = true;
		for (std::vector<cv::Point2f>::size_type j = 0; j < undistortedFeatures2D.size(); j++) {
			if (cv::norm(
					featuresSandBoxUndistorted[i] - undistortedFeatures2D[j])
					< matcherParameters.OpenCVParams.minimalReprojDistanceNewTrackingFeatures) {
				addFeature = false;
				break;
			}
		}
		if (addFeature) {
			undistortedFeatures2D.push_back(featuresSandBoxUndistorted[i]);
			distortedFeatures2D.push_back(featuresSandBoxDistorted[i]);
			features3D.push_back(features3DSandBox[i]);
			keyPoints.push_back(keyPointsSandBox[i]);
			detDists.push_back(detDistsSandBox[i]);
		}
	}

}

// Tracking features
double Matcher::trackKLT(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Current 2D positions, 3D positions and found matches
	//std::vector<cv::Point2f> undistortedFeatures2D(prevFeaturesUndistorted), distortedFeatures2D;
	std::vector<cv::Point2f> undistortedFeatures2D, distortedFeatures2D;
	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keyPoints;
    std::vector<double> detDists;
	std::vector<Eigen::Vector3f> features3D;
	std::vector<cv::DMatch> matches;

	// No features so identity()
	if (prevFeaturesUndistorted.size() == 0 || prevFeaturesDistorted.size() == 0) {
		estimatedTransformation = Eigen::Matrix4f::Identity();
	} else {
		// Tracking features and creating potential matches
		matches = performTracking(prevRgbImage, sensorData.rgbImage,
						prevFeaturesDistorted, distortedFeatures2D,
						prevKeyPoints,
						keyPoints,
						prevDetDists,
						detDists);

		// Remove distortion
		undistortedFeatures2D = RGBD::removeImageDistortion(distortedFeatures2D,
					matcherParameters.cameraMatrixMat,
					matcherParameters.distortionCoeffsMat);

		// Check that we have the same number of undistorted and distorted features
		assert(("TrackKLT: distorted vs undistorted sizes", undistortedFeatures2D.size()
								== distortedFeatures2D.size()));

		// Visualize matches
		if (matcherParameters.verbose > 1)
		{
			std::vector<cv::KeyPoint> featuresToShow[2];
			cv::KeyPoint::convert(prevFeaturesDistorted, featuresToShow[0]);
			cv::KeyPoint::convert(distortedFeatures2D, featuresToShow[1]);
			showMatches(prevRgbImage, featuresToShow[0], sensorData.rgbImage, featuresToShow[1],
					matches);

		}
		// Show detected features
		if (matcherParameters.verbose > 2) {
//			cv::KeyPoint::convert(distortedFeatures2D, prevKeyPoints);
			showFeatures(sensorData.rgbImage, prevKeyPoints);
		}

		// Associate depth -> creating 3D features
		features3D = RGBD::keypoints2Dto3D(undistortedFeatures2D,
				sensorData.depthImage, matcherParameters.cameraMatrixMat,
				sensorData.depthImageScale);

		// Remove features based on 2D and 3D distance
		if ( matcherParameters.OpenCVParams.removeTooCloseFeatures > 0)
			removeTooCloseFeatures(distortedFeatures2D,
									undistortedFeatures2D,
									features3D,
									keyPoints,
									detDists,
									matches);

		// Checking that sizes are correct
		assert(
				("TrackKLT: After tracking: 2D and 3D sizes", undistortedFeatures2D.size()
						== features3D.size()));

		// Setting the version of RANSAC
		matcherParameters.RANSACParams.errorVersion =
				matcherParameters.RANSACParams.errorVersionVO;

		// Creating RANSAC and running it
		RANSAC ransac(matcherParameters.RANSACParams,
				matcherParameters.cameraMatrixMat);
		estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
				features3D, matches, inlierMatches);
	}

	// If the number of tracked features falls below certain number, we detect new features are merge them together
	if ((int)undistortedFeatures2D.size()
			< matcherParameters.OpenCVParams.minimalTrackedFeatures) {

		// Detect new salient features
		std::vector<cv::KeyPoint> keyPointsSandbox = detectFeatures(
				sensorData.rgbImage);

		// DBScan on detected features to remove groups of points
		DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
		dbscan.run(keyPointsSandbox);

		// Extract 2D points from keypoints
		std::vector<cv::Point2f> featuresSandBoxDistorted;
		cv::KeyPoint::convert(keyPointsSandbox, featuresSandBoxDistorted);

		// Find 2D positions without distortion
		std::vector<cv::Point2f> featuresSandBoxUndistorted =
				RGBD::removeImageDistortion(keyPointsSandbox,
						matcherParameters.cameraMatrixMat,
						matcherParameters.distortionCoeffsMat);

		std::vector<Eigen::Vector3f> features3DSandbox = RGBD::keypoints2Dto3D(
						featuresSandBoxUndistorted, sensorData.depthImage,
						matcherParameters.cameraMatrixMat, sensorData.depthImageScale);

        std::vector<double> detDistsSandbox;
		for(std::vector<Eigen::Vector3f>::size_type i = 0; i < features3DSandbox.size(); ++i){
            double dist = std::sqrt(features3DSandbox[i][0]*features3DSandbox[i][0] +
										features3DSandbox[i][1]*features3DSandbox[i][1] +
										features3DSandbox[i][2]*features3DSandbox[i][2]);

			detDistsSandbox.push_back(dist);
		}

		// Merging features - rejecting feature too close to existing ones
		// Parameters: (existing features and vector to add new features, new features, minimal image distance between features)
        //std::cout<<"We had " << distortedFeatures2D.size() << std::endl;
		mergeTrackedFeatures(undistortedFeatures2D,
							featuresSandBoxUndistorted,
							distortedFeatures2D,
							featuresSandBoxDistorted,
							features3D,
							features3DSandbox,
							keyPoints,
							keyPointsSandbox,
							detDists,
							detDistsSandbox);
        //std::cout<<"After merging " << distortedFeatures2D.size() << std::endl;

//		// Add depth to new features
//		std::vector<Eigen::Vector3f> newFeatures3D = RGBD::keypoints2Dto3D(
//				undistortedFeatures2D, sensorData.depthImage,
//				matcherParameters.cameraMatrixMat, sensorData.depthImageScale,
//				features3D.size());
//
//		// Merge 3D positions of old and new features
//		features3D.reserve(features3D.size() + newFeatures3D.size());
//		features3D.insert(features3D.end(), newFeatures3D.begin(),
//				newFeatures3D.end());

		// Remove features based on 2D and 3D distance
		if (matcherParameters.OpenCVParams.removeTooCloseFeatures > 0)
			removeTooCloseFeatures(distortedFeatures2D, undistortedFeatures2D,
					features3D, keyPoints, detDists, matches);

	}

	std::vector<cv::KeyPoint> descKeyPoints = keyPoints;
	//Compute predicted scale
	for (std::vector<cv::KeyPoint>::size_type i = 0; i < descKeyPoints.size();
			++i) {
		int detLevel = descKeyPoints[i].octave;
		double detLevelScaleFactor = pow(scaleFactor, detLevel);
		double curDist = std::sqrt(
				features3D[i][0] * features3D[i][0]
						+ features3D[i][1] * features3D[i][1]
						+ features3D[i][2] * features3D[i][2]);
		double curLevelScaleFactor = detLevelScaleFactor * detDists[i]
				/ curDist;

		//To compute log_{scaleFactor}(curLevelScaleFactor) = log_{e}{curLevelScaleFactor} / log_{e}(scaleFactor)
		int curLevel = (int) std::ceil(
				std::log(curLevelScaleFactor) / logScaleFactor);
		curLevel = std::max(0, curLevel);
		curLevel = std::min(nLevels - 1, curLevel);
		descKeyPoints[i].octave = curLevel;
//			descKeyPoints[i].octave = 0;
	}
	{
		//TODO opencv's ORB::compute sorts keyPoints according to octave,
		// to prevent this we sort it earlier and maintain correct order in all structures
		std::vector<std::vector<std::pair<int, int>>>keyPointsLevel(nLevels);
		for(std::vector<cv::KeyPoint>::size_type i = 0; i < keyPoints.size(); ++i) {
			//use current level
			int level = descKeyPoints[i].octave;
			keyPointsLevel[level].emplace_back(level, i);
		}
		std::vector<cv::Point2f> tmpDistorted, tmpUndistorted;
		std::vector<Eigen::Vector3f> tmp3D;
		std::vector<cv::KeyPoint> tmpKeyPoints;
		std::vector<double> tmpDetDists;

		for(std::vector<int>::size_type l = 0; l < keyPointsLevel.size(); ++l) {
			for(std::vector<int>::size_type i = 0; i < keyPointsLevel[l].size(); ++i) {
				tmpDistorted.push_back(distortedFeatures2D[keyPointsLevel[l][i].second]);
				tmpUndistorted.push_back(undistortedFeatures2D[keyPointsLevel[l][i].second]);
				tmp3D.push_back(features3D[keyPointsLevel[l][i].second]);
				tmpKeyPoints.push_back(keyPoints[keyPointsLevel[l][i].second]);
				tmpDetDists.push_back(detDists[keyPointsLevel[l][i].second]);
			}
		}

		tmpDistorted.swap(distortedFeatures2D);
		tmpUndistorted.swap(undistortedFeatures2D);
		tmp3D.swap(features3D);
		tmpKeyPoints.swap(keyPoints);
		tmpDetDists.swap(detDists);
	}

//		auto endA = std::chrono::high_resolution_clock::now();
//		std::cout << "\tLiczenie skal = " << std::chrono::duration_cast<std::chrono::milliseconds>(endA - startA).count() << " ms"<< std::endl;

	// Computing descriptors
//		auto startC = std::chrono::high_resolution_clock::now();
	descriptors = describeFeatures(sensorData.rgbImage, descKeyPoints);
//		auto endC = std::chrono::high_resolution_clock::now();
//		std::cout << "\tDeskrypcja = " << std::chrono::duration_cast<std::chrono::milliseconds>(endC - startC).count() << " ms"<< std::endl;

	// Some unlucky case --> couldn't describe a feature, so we need to remove it and recompute 3D positions
	if (descKeyPoints.size() != keyPoints.size()) {

//			std::cout << "descKeyPoints.size() = " << descKeyPoints.size() << std::endl;
//			std::cout << "undistortedFeatures2D.size() = " << undistortedFeatures2D.size() << std::endl;
//			std::cout << "descriptors.rows = " << descriptors.rows << std::endl;
//
//			for(int i = 0; i < descKeyPoints.size(); ++i){
//				std::cout << descKeyPoints[i].pt << "(" << descKeyPoints[i].octave << ")" <<
//					"\t" << keyPoints[i].pt << "(" << keyPoints[i].octave << ")" << std::endl;
//			}

		std::vector<cv::Point2f> tmpDistorted, tmpUndistorted;
		std::vector<Eigen::Vector3f> tmp3D;
		std::vector<cv::KeyPoint> tmpKeyPoints;
		std::vector<double> tmpDetDists;

		for (std::vector<cv::KeyPoint>::size_type i = 0, j = 0;
				i < keyPoints.size(); i++) {
//				std::cout << "i = " << i << ", j = " << j << std::endl;
			if (j == descKeyPoints.size())
				break;
//				std::cout << "keyPoints[i].pt = " << keyPoints[i].pt << std::endl;
//				std::cout << "descKeyPoints[j].pt = " << descKeyPoints[j].pt << std::endl;
			if (cv::norm(keyPoints[i].pt - descKeyPoints[j].pt) < 0.0001) {
//					std::cout << "match" << std::endl;
				tmpDistorted.push_back(distortedFeatures2D[i]);
				tmpUndistorted.push_back(undistortedFeatures2D[i]);
				tmp3D.push_back(features3D[i]);
				tmpKeyPoints.push_back(keyPoints[i]);
				tmpDetDists.push_back(detDists[i]);
				j++;
			}
		}
		tmpDistorted.swap(distortedFeatures2D);
		tmpUndistorted.swap(undistortedFeatures2D);
		tmp3D.swap(features3D);
		tmpKeyPoints.swap(keyPoints);
		tmpDetDists.swap(detDists);
	}

//	auto end = std::chrono::high_resolution_clock::now();
//	std::cout << "\tDESC = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms"<< std::endl;

	// Check that the sizes are ok
	assert(
			("TrackKLT: 2D and 3D sizes at the end: featuresUndistorted", distortedFeatures2D.size()
					== undistortedFeatures2D.size()));
	assert(
			("TrackKLT: 2D and 3D sizes at the end: features3d", distortedFeatures2D.size()
					== features3D.size()));
	assert(
				("TrackKLT: 2D and 3D sizes at the end: descriptors", distortedFeatures2D.size()
						== descriptors.rows));
	assert(
			("TrackKLT: 2D and 3D sizes at the end: keyPoints", distortedFeatures2D.size()
					== keyPoints.size()));

	assert(
			("TrackKLT: 2D and 3D sizes at the end: detDists", distortedFeatures2D.size()
					== detDists.size()));

	double inlierRatio = 0.0;
	if ( matches.size() > 0)
		inlierRatio = RANSAC::pointInlierRatio(inlierMatches, matches);

//	if ( inlierRatio < 0.1 )
//	{
//		std::cout << "inlierRatio = " << inlierRatio << " Feature sizes: "
//				<< prevFeaturesDistorted.size() << " "
//				<< distortedFeatures2D.size() << std::endl;
//		std::vector<cv::KeyPoint> tmp[2];
//		cv::KeyPoint::convert(prevFeaturesDistorted, tmp[0]);
//		cv::KeyPoint::convert(distortedFeatures2D, tmp[1]);
//		showFeatures(prevRgbImage, tmp[0]);
//		showFeatures(sensorData.rgbImage, tmp[1]);
//	}

	if(distortedFeatures2D.size() != undistortedFeatures2D.size() ||
			distortedFeatures2D.size() != features3D.size() ||
			(int)distortedFeatures2D.size() != descriptors.rows ||
			distortedFeatures2D.size() != keyPoints.size() ||
			distortedFeatures2D.size() != detDists.size())
	{
		std::cout << "Bad number of entries" << std::endl;
		std::cout << "distortedFeatures2D.size() = " << distortedFeatures2D.size() << std::endl;
		std::cout << "undistortedFeatures2D.size() = " << undistortedFeatures2D.size() << std::endl;
		std::cout << "features3D.size() = " << features3D.size() << std::endl;
		std::cout << "descriptors.rows = " << descriptors.rows << std::endl;
		std::cout << "keyPoints.size() = " << keyPoints.size() << std::endl;
		std::cout << "detDists.size() = " << detDists.size() << std::endl;
	}

	// Save computed values for next iteration
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	distortedFeatures2D.swap(prevFeaturesDistorted);
	features3D.swap(prevFeatures3D);
	prevDescriptors = descriptors;
	keyPoints.swap(prevKeyPoints);
	detDists.swap(prevDetDists);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;


	return inlierRatio;
}

// Vanilla feature matching from OpenCV; Used in VO with matching
double Matcher::match(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Detect salient features
	std::vector<cv::KeyPoint> keyPoints = detectFeatures(sensorData.rgbImage);

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
	dbscan.run(keyPoints);

	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, keyPoints);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(sensorData.rgbImage, keyPoints);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Find 2D positions without distortion
	std::vector<cv::Point2f> undistortedFeatures2D =
			RGBD::removeImageDistortion(keyPoints,
					matcherParameters.cameraMatrixMat,
					matcherParameters.distortionCoeffsMat);

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(
			undistortedFeatures2D, sensorData.depthImage,
			matcherParameters.cameraMatrixMat, sensorData.depthImageScale);

	// Visualize matches
	if (matcherParameters.verbose > 1)
		showMatches(prevRgbImage, prevKeyPoints, sensorData.rgbImage, keyPoints,
				matches);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	matcherParameters.RANSACParams.errorVersion =
			matcherParameters.RANSACParams.errorVersionVO;
	RANSAC ransac(matcherParameters.RANSACParams,
			matcherParameters.cameraMatrixMat);

	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
			features3D, matches, inlierMatches);

	// Check
	assert(("Match sizes", keyPoints.size() == undistortedFeatures2D.size()));
	assert(
			("Match sizes 2", undistortedFeatures2D.size() == features3D.size()));
	assert(("Match sizes 3", features3D.size() == descriptors.rows));

	// Save computed values for next iteration
	keyPoints.swap(prevKeyPoints);
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	features3D.swap(prevFeatures3D);
	cv::swap(descriptors, prevDescriptors);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;

	return RANSAC::pointInlierRatio(inlierMatches, matches);
}

// We have chosen to use the first descriptor. TODO: It always chooses first descriptor!
cv::Mat Matcher::extractMapDescriptors(std::vector<MapFeature> mapFeatures) {
	cv::Mat mapDescriptors;
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it) {
		mapDescriptors.push_back(it->descriptors.begin()->second.descriptor);
	}
	return mapDescriptors;
}

// Again, need to extract the Vec3 position of feature to reasonable format -> using nice std::algorithm
// TODO: Not needed after change to Vec3
std::vector<Eigen::Vector3f> Matcher::extractMapFeaturesPositions(
		std::vector<MapFeature> mapFeatures) {

	std::vector<Eigen::Vector3f> mapFeaturePositions3D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
			mapFeaturePositions3D.begin(),
			[](const MapFeature& m) {return m.position.vector().cast<float>();});
	return mapFeaturePositions3D;
}


// Guided matching for detected keypoints to mapFeatures
double Matcher::matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation, bool newDetection,
		std::vector<int> frameIds, int computationNumber) {

	if (!newDetection)
		return matchXYZ(mapFeatures,
						sensorPoseId,
						foundInlierMapFeatures,
						estimatedTransformation,
						prevDescriptors,
						prevFeatures3D,
						prevKeyPoints,
						prevDetDists,
						frameIds, computationNumber);

	// Detect salient features
	prevKeyPoints = detectFeatures(prevRgbImage);

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
	dbscan.run(prevKeyPoints);

	// Show detected features
	if (matcherParameters.verbose > 2)
		showFeatures(prevRgbImage, prevKeyPoints);

	prevDescriptors = describeFeatures(prevRgbImage, prevKeyPoints);

	// Extract 2D points from keypoints
	cv::KeyPoint::convert(prevKeyPoints, prevFeaturesDistorted);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevKeyPoints,
			matcherParameters.cameraMatrixMat,
			matcherParameters.distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted,
			prevDepthImage, matcherParameters.cameraMatrixMat,
			prevDepthImageScale);

	prevDetDists.clear();
	for(std::vector<Eigen::Vector3f>::size_type i = 0; i < prevFeatures3D.size(); ++i){
        double dist = std::sqrt(prevFeatures3D[i][0]*prevFeatures3D[i][0] +
									prevFeatures3D[i][1]*prevFeatures3D[i][1] +
									prevFeatures3D[i][2]*prevFeatures3D[i][2]);

		prevDetDists.push_back(dist);
	}


	return matchXYZ(mapFeatures,
			sensorPoseId,
			foundInlierMapFeatures,
			estimatedTransformation,
			prevDescriptors,
			prevFeatures3D,
			prevKeyPoints,
			prevDetDists,
			frameIds, computationNumber);
}

// Guided matching
double Matcher::matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation,
		cv::Mat currentPoseDescriptors,
		std::vector<Eigen::Vector3f> &currentPoseFeatures3D,
		std::vector<cv::KeyPoint>& currentPoseKeyPoints,
        std::vector<double>& currentPoseDetDists,
		std::vector<int> frameIds,
		int computationNumber)
{

	double matchingXYZSphereRadius = matcherParameters.OpenCVParams.matchingXYZSphereRadius;
	double matchingXYZacceptRatioOfBestMatch = matcherParameters.OpenCVParams.matchingXYZacceptRatioOfBestMatch;
	if (computationNumber > 1) {
		matchingXYZSphereRadius += 0.02 * (computationNumber - 1);
		matchingXYZacceptRatioOfBestMatch = std::max(0.1, matchingXYZacceptRatioOfBestMatch - 0.05 *(computationNumber - 1));
	}


	int normType = cv::NORM_HAMMING;
	if (matcherParameters.OpenCVParams.descriptor == "SURF"
				|| matcherParameters.OpenCVParams.descriptor == "SIFT")
		normType = cv::NORM_L2;

	// Check some asserts
	assert(
			("matchXYZ: 2D and 3D sizes", prevDescriptors.rows
					== prevFeaturesUndistorted.size()));
	assert(
			("matchXYZ: 2D and 3D sizes 2", prevDescriptors.rows
					== prevFeatures3D.size()));


	std::vector<int> currentPosePredLevels(currentPoseKeyPoints.size());
	//Compute predicted scale
	for(std::vector<cv::KeyPoint>::size_type i = 0; i < currentPoseKeyPoints.size(); ++i){
		int detLevel = currentPoseKeyPoints[i].octave;
        double detLevelScaleFactor = pow(scaleFactor, detLevel);
        double curDist = currentPoseFeatures3D[i].norm();
        double curLevelScaleFactor = detLevelScaleFactor * currentPoseDetDists[i] / curDist;

		//To compute log_{scaleFactor}(curLevelScaleFactor) = log_{e}{curLevelScaleFactor} / log_{e}(scaleFactor)
		int curLevel = (int)std::ceil(std::log(curLevelScaleFactor) / logScaleFactor);
		curLevel = std::max(0, curLevel);
		curLevel = std::min(nLevels - 1, curLevel);
		currentPosePredLevels[i] = curLevel;

//		std::cout << "currentPoseKeyPoint[" << i << "]" << std::endl;
//		std::cout << "detDist = " << currentPoseDetDists[i] << std::endl;
//		std::cout << "detLevel = " << detLevel << std::endl;
//		std::cout << "curDist = " << curDist << std::endl;
//		std::cout << "curLevel = " << curLevel << std::endl;
	}


	// Perform matching
	std::vector<cv::DMatch> matches;

	// We need to extract positions from vector<class> to independent vectors to use OpenCV functions
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// For all features in the map
	int j = 0, perfectMatchCounter = 0;
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it, ++j) {


		// Find the closest view in a map for a feature
		ExtendedDescriptor extDesc;
		if (frameIds.size() > 0)
			extDesc = it->descriptors[frameIds[j]];
		else
			extDesc = it->descriptors.begin()->second;

		//Compute predicted scale
		int& detLevel = extDesc.octave;
        double detLevelScaleFactor = pow(scaleFactor, detLevel);
        double& detDist = extDesc.detDist;
        double curDist = std::sqrt(it->position.vector()[0]*it->position.vector()[0] +
										it->position.vector()[1]*it->position.vector()[1] +
										it->position.vector()[2]*it->position.vector()[2]);
        double curLevelScaleFactor = detLevelScaleFactor * detDist / curDist;
		//To compute log_{scaleFactor}(curLevelScaleFactor) = log_{e}{curLevelScaleFactor} / log_{e}(scaleFactor)
		int curLevel = (int)std::ceil(std::log(curLevelScaleFactor) / logScaleFactor);
		curLevel = std::max(0, curLevel);
		curLevel = std::min(nLevels - 1, curLevel);


		// Possible matches for considered feature
		std::vector<int> possibleMatchId;

		// Reject all matches that are further away than threshold
		for (std::vector<ExtendedDescriptor>::size_type i = 0; i < currentPoseFeatures3D.size(); i++) {
			Eigen::Vector3f tmp((float) it->position.x(),
					(float) it->position.y(), (float) it->position.z());
			float norm = (tmp - (currentPoseFeatures3D[i])).norm();

			bool scaleCheck = currentPosePredLevels[i] - 1 <= curLevel &&
								curLevel <= currentPosePredLevels[i] + 1;
//			bool scaleCheck = true;
			bool posCheck = norm < matchingXYZSphereRadius;
			if (posCheck && scaleCheck) {
				possibleMatchId.push_back((int)i);
			}
		}

		// Find best match based on descriptors
		int bestId = -1;
		float bestVal = 99999;
		for (std::vector<int>::size_type i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x = (extDesc.descriptor
					- currentPoseDescriptors.row(id));
			float value = (float)norm(x, normType);
			if (value < bestVal || bestId == -1) {
				bestVal = value;
				bestId = id;
			}
		}

		// Pretty nice match
		if (bestVal < 0.1) {
			perfectMatchCounter++;
		}

		// Check the rest compared to the best
		for (std::vector<int>::size_type i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x = (extDesc.descriptor
					- currentPoseDescriptors.row(id));
			float value = (float)norm(x, normType);
			if (matchingXYZacceptRatioOfBestMatch * value <= bestVal) {
				cv::DMatch tmpMatch;
				tmpMatch.distance = value;
				tmpMatch.queryIdx = j;
				tmpMatch.trainIdx = id;
				matches.push_back(tmpMatch);
			}
		}
	}

	if (matcherParameters.verbose > 0)
		std::cout << "MatchesXYZ - we found : " << matches.size()
				<< " (Perfect matches = " << perfectMatchCounter << ")"
				<< std::endl;

	if (matches.size() <= 0)
		return -1.0;

	// Choosing RANSAC version
	std::vector<cv::DMatch> inlierMatches;
	matcherParameters.RANSACParams.errorVersion =
			matcherParameters.RANSACParams.errorVersionMap;

	// Creating and estimating transformation
	RANSAC ransac(matcherParameters.RANSACParams,
			matcherParameters.cameraMatrixMat);
	estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, currentPoseFeatures3D, matches, inlierMatches);

	// for all inliers, convert them to map-compatible format
	foundInlierMapFeatures.clear();
	for (std::vector<cv::DMatch>::iterator it = inlierMatches.begin();
			it != inlierMatches.end(); ++it) {
		int mapId = it->queryIdx, currentPoseId = it->trainIdx;

		MapFeature mapFeature;
		mapFeature.id = mapFeatures[mapId].id;

		mapFeature.u = prevFeaturesUndistorted[currentPoseId].x;
		mapFeature.v = prevFeaturesUndistorted[currentPoseId].y;

		mapFeature.position = Vec3(
				currentPoseFeatures3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);

		ExtendedDescriptor featureExtendedDescriptor(prevFeaturesUndistorted[currentPoseId],
				prevFeaturesDistorted[currentPoseId], mapFeature.position,
				currentPoseDescriptors.row(currentPoseId),
				currentPoseKeyPoints[currentPoseId].octave,
				currentPoseDetDists[currentPoseId]);
		mapFeature.descriptors[sensorPoseId]=featureExtendedDescriptor;

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}

	// Compute inlier ratio
	return RANSAC::pointInlierRatio(inlierMatches, matches);
}


// Matching in case of loop closure
double Matcher::matchFeatureLoopClosure(std::vector<MapFeature> featureSets[2], int framesIds[2], std::vector<std::pair<int, int>> &pairedFeatures,
		Eigen::Matrix4f &estimatedTransformation){

	cv::Mat extractedDescriptors[2];
	std::vector<cv::Point2f> points2D[2];
	std::vector<Eigen::Vector3f> points3D[2];

	for (int i = 0; i < 2; i++) {
		std::vector<MapFeature> &analyzedSet = featureSets[i];
		int &currentFrameId = framesIds[i];

		for (auto& feature : analyzedSet) {

			// We look for the descriptor and u,v from a position we currently analyze
			ExtendedDescriptor &ext = feature.descriptors[currentFrameId];

			points2D[i].push_back(ext.point2DUndist);
			points3D[i].push_back(
					Eigen::Vector3f((float) ext.point3D.x(),
							(float) ext.point3D.y(), (float) ext.point3D.z()));
			feature.u = ext.point2DUndist.x;
			feature.v = ext.point2DUndist.y;
			extractedDescriptors[i].push_back(ext.descriptor);

		}
	}

	// Sanity check - it should have been already checked
	if ( points2D[0].size() < 10 || points2D[1].size() < 10)
	{
		std::cout<<"Too few features :(" << std::endl;
		return 0;
	}
	std::vector<cv::DMatch> matches = performMatching(extractedDescriptors[0],
			extractedDescriptors[1]);

	if (matches.size() <= 0)
		return -1.0;

	// Choosing RANSAC version
	std::vector<cv::DMatch> inlierMatches;
	matcherParameters.RANSACParams.errorVersion =
			matcherParameters.RANSACParams.errorVersionMap;

	// Creating and estimating transformation
	RANSAC ransac(matcherParameters.RANSACParams,
			matcherParameters.cameraMatrixMat);
	estimatedTransformation = ransac.estimateTransformation(
			points3D[0], points3D[1], matches,
			inlierMatches);

	// Repack DMatch to pair<int,int> of matched features
	pairedFeatures.clear();
	for (auto &match : inlierMatches) {
		pairedFeatures.push_back(std::make_pair(match.queryIdx, match.trainIdx));
	}

	return RANSAC::pointInlierRatio(inlierMatches, matches);

}


/// Helper methods:
void Matcher::showFeatures(cv::Mat rgbImage,
		std::vector<cv::KeyPoint> featuresToShow) {
	cv::Mat imageToShow;
	cv::drawKeypoints(rgbImage, featuresToShow, imageToShow);

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

std::set<int> Matcher::removeTooCloseFeatures(std::vector<cv::Point2f>& distortedFeatures2D,
		std::vector<cv::Point2f>& undistortedFeatures2D,
		std::vector<Eigen::Vector3f> &features3D,
		std::vector<cv::KeyPoint>& keyPoints,
        std::vector<double>& detDists,
		std::vector<cv::DMatch> &matches)
{

	// Check that we have the same sizes
	assert(("removeTooCloseFeatures: distorted vs undistorted sizes", undistortedFeatures2D.size()
							== distortedFeatures2D.size()));
	assert(("removeTooCloseFeatures: features3D vs undistorted sizes", features3D.size()
								== distortedFeatures2D.size()));

	std::set<int> featuresToRemove;

	for (std::vector<Eigen::Vector3f>::size_type i = 0; i < features3D.size(); i++) {
		for (std::vector<Eigen::Vector3f>::size_type j = i + 1; j < features3D.size(); j++) {

			double x = features3D[i][0] - features3D[j][0];
			double y = features3D[i][1] - features3D[j][1];
			double z = features3D[i][2] - features3D[j][2];
			double dist3D = sqrt(x * x + y * y + z * z);

			double u = undistortedFeatures2D[i].x - undistortedFeatures2D[j].x;
			double v = undistortedFeatures2D[i].y - undistortedFeatures2D[j].y;
			double dist2D = sqrt(u * u + v * v);

			if ( dist3D < matcherParameters.OpenCVParams.minimalEuclidDistanceNewTrackingFeatures ||
					dist2D < matcherParameters.OpenCVParams.minimalReprojDistanceNewTrackingFeatures) {
				featuresToRemove.insert((int)j); // TODO: Arbitrary decision right now
			}
		}
	}

	// Removing from distorted Features
	int count = 0;
	distortedFeatures2D.erase(
			std::remove_if(distortedFeatures2D.begin(),
					distortedFeatures2D.end(),
					[&](const cv::Point2f & /*o*/) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			distortedFeatures2D.end());

	// Removing from undistorted Features
	count = 0;
	undistortedFeatures2D.erase(
			std::remove_if(undistortedFeatures2D.begin(),
					undistortedFeatures2D.end(),
					[&](const cv::Point2f & /*o*/) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			undistortedFeatures2D.end());

	// Removing from features 3D
	count = 0;
	features3D.erase(
			std::remove_if(features3D.begin(), features3D.end(),
					[&](const Eigen::Vector3f & /*o*/) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			features3D.end());

	// Removing from key points
	count = 0;
	keyPoints.erase(
			std::remove_if(keyPoints.begin(), keyPoints.end(),
					[&](const cv::KeyPoint & /*o*/) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			keyPoints.end());

	// Removing from detection distance
	count = 0;
	detDists.erase(
			std::remove_if(detDists.begin(), detDists.end(),
                    [&](const double & /*o*/) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			detDists.end());


	// Removing from matches
	matches.erase(
			std::remove_if(matches.begin(), matches.end(),
					[&](const cv::DMatch & o) {return featuresToRemove.find(o.trainIdx) != featuresToRemove.end();}),
				matches.end());

	// Check that we have the same sizes
	assert(
			("After removeTooCloseFeatures: distorted vs undistorted sizes", undistortedFeatures2D.size()
					== distortedFeatures2D.size()));
	assert(
			("After removeTooCloseFeatures: features3D vs undistorted sizes", features3D.size()
					== distortedFeatures2D.size()));

	return featuresToRemove;
}

int Matcher::getNumberOfFeatures() {
	return (int)prevFeaturesDistorted.size();
}

Matcher::featureSet Matcher::getFeatures() {
	featureSet returnSet;
	returnSet.descriptors = prevDescriptors;
	returnSet.feature2D = prevKeyPoints;
	returnSet.detDist = prevDetDists;
	returnSet.distortedFeature2D = prevFeaturesDistorted;
	returnSet.undistortedFeature2D = prevFeaturesUndistorted;
	returnSet.feature3D = prevFeatures3D;
	return returnSet;
}

///// Run the match with two poses from the map. Parameters:
///// 	featureSet[2] 	-> set of features from the first and the second pose.
/////			Remark: It only makes sense when features are moved to the local
/////					coordinate system of features before calling the function
/////		frameIds[2] 	-> set of ids of poses for features with the closest angle of observations -
/////					in other words for each descriptor, we get the id of the pose so we can take the apprioprate descriptor
///// 	pairedFeatures 	-> return pairs of ids of features that are matched
/////		estimatedTransformation		-> the estimated transformation
//double Matcher::matchPose2Pose(std::vector<MapFeature> featureSet[2],
//		std::vector<int> /*frameIds*/[2],
//		std::vector<std::pair<int, int>> &/*pairedFeatures*/,
//		Eigen::Matrix4f &estimatedTransformation) {
//
//
//	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
//    cv::Mat descriptors[2] = { extractMapDescriptors(featureSet[0]),
//    		extractMapDescriptors(featureSet[1]) };
//
//	std::vector<Eigen::Vector3f> featurePositions3D[2] = {
//			extractMapFeaturesPositions(featureSet[0]),
//			extractMapFeaturesPositions(featureSet[1]) };
//
//	std::vector<cv::DMatch> matches = performMatching(descriptors[0], descriptors[1]);
//
//
//    if (matcherParameters.verbose > 0)
//		std::cout << "MatchPose2Pose - we found : " << matches.size() << std::endl;
//
//
//	if (matches.size() <= 0)
//		return -1.0;
//
//	// Choosing RANSAC version
//	std::vector<cv::DMatch> inlierMatches;
//	matcherParameters.RANSACParams.errorVersion =
//			matcherParameters.RANSACParams.errorVersionMap;
//
//	// Creating and estimating transformation
//	RANSAC ransac(matcherParameters.RANSACParams,
//			matcherParameters.cameraMatrixMat);
//	estimatedTransformation = ransac.estimateTransformation(
//			featurePositions3D[0], featurePositions3D[1], matches,
//			inlierMatches);
//
//	return RANSAC::pointInlierRatio(inlierMatches, matches);
//}
//
///// Run the match with two poses from the map. Parameters:
///// 	featureSet[2] 	-> set of features from the first and the second pose.
/////			Remark: It only makes sense when features are moved to the local
/////					coordinate system of features before calling the function
///// 	pairedFeatures 	-> return pairs of ids of features that are matched
/////		estimatedTransformation		-> the estimated transformation
//double Matcher::matchPose2Pose(std::vector<MapFeature> featureSet[2],
//		std::vector<std::pair<int, int>> &pairedFeatures,
//		Eigen::Matrix4f &estimatedTransformation) {
//
//	// Place to store the poseIds of the chosen descriptors
//	std::vector<int> frameIds[2];
//
//	// For both poses
//	for (int i = 0; i < 2; i++) {
//
//		// We resize the vectors to have the same size as sets of features
//		frameIds[i].resize(featureSet[i].size(), 0);
//
//		// For every feature, we save the original pose id (the id of the first descriptor)
//        int j = 0;
//		for (std::vector<MapFeature>::iterator it = featureSet[i].begin();
//                it != featureSet[i].end(); ++it, ++j) {
//			frameIds[i][j] = it->descriptors[0].poseId;
//        }
//	}
//
//    // Now, we can all the more complex version
//    return matchPose2Pose(featureSet, frameIds, pairedFeatures, estimatedTransformation);
//}
//
//// Versions as above, but works on two sensor frames (it performs detection on its own)
//double Matcher::matchPose2Pose(SensorFrame sensorFrames[2],
//		Eigen::Matrix4f &estimatedTransformation) {
//
//	cv::Mat wtf[2];
//	std::vector<Eigen::Vector3f> features3D[2];
//	std::vector<cv::KeyPoint> features[2];
//
//	std::vector<MapFeature> featureSet[2];
//	for (int i = 0; i < 2; i++) {
//		// Detect salient features
//		features[i] = detectFeatures(
//				sensorFrames[i].rgbImage);
//
////		// DBScan
////		DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
////		dbscan.run(features);
//
//		// Describe salient features
//		cv::Mat descriptors = describeFeatures(sensorFrames[i].rgbImage, features[i]);
//
//		// Find 2D positions with distortion
//		std::vector<cv::Point2f> distortedFeatures2D;
//		cv::KeyPoint::convert(features[i],distortedFeatures2D);
//
//		// Find 2D positions without distortion
//		std::vector<cv::Point2f> undistortedFeatures2D =
//				RGBD::removeImageDistortion(features[i],
//						matcherParameters.cameraMatrixMat,
//						matcherParameters.distortionCoeffsMat);
//
//		// Associate depth
//		features3D[i] = RGBD::keypoints2Dto3D(undistortedFeatures2D,
//				sensorFrames[i].depthImage, matcherParameters.cameraMatrixMat,
//				sensorFrames[i].depthImageScale);
//
//		//Compute distance at which feature was detected
//		std::vector<double> detDists(features3D[i].size());
//		for(std::vector<Eigen::Vector3f>::size_type j = 0; j < features3D[i].size(); ++j){
//			double dist = std::sqrt(features3D[i][j][0]*features3D[i][j][0] +
//										features3D[i][j][1]*features3D[i][j][1] +
//										features3D[i][j][2]*features3D[i][j][2]);
//			detDists[j] = dist;
//		}
//
//		// Convert to map format
//		for (std::vector<Eigen::Vector3f>::size_type j=0;j<features3D[i].size();j++) {
//			MapFeature mapFeature;
//			mapFeature.id = (unsigned int)j;
//
//			mapFeature.u = undistortedFeatures2D[j].x;
//			mapFeature.v = undistortedFeatures2D[j].y;
//
//			mapFeature.position = Vec3(
//					features3D[i][j].cast<double>());
//			mapFeature.posesIds.push_back(i);
//
//			ExtendedDescriptor featureExtendedDescriptor(i,
//					distortedFeatures2D[j],
//					undistortedFeatures2D[j],
//					Vec3(features3D[i][j].x(), features3D[i][j].y(), features3D[i][j].z()),
//					descriptors.row((int) j),
//					features[i][j].octave,
//					detDists[j]);
//			mapFeature.descriptors.push_back(featureExtendedDescriptor);
//
//			// Add the measurement
//			featureSet[i].push_back(mapFeature);
//        }
//
//		wtf[i] = descriptors;
//	}
//
//	std::vector<cv::DMatch> matches = performMatching(wtf[0], wtf[1]), inlierMatches;
//
////	showMatches(sensorFrames[0].rgbImage, features[0], sensorFrames[1].rgbImage, features[1],
////							matches);
//
//	matcherParameters.RANSACParams.errorVersion =
//				matcherParameters.RANSACParams.errorVersionVO;
//	RANSAC ransac(matcherParameters.RANSACParams,
//				matcherParameters.cameraMatrixMat);
//
//	estimatedTransformation = ransac.estimateTransformation(features3D[0], features3D[1], matches, inlierMatches);
//
//	//std::cout<<"WHY MATCHING IS IRRITATING : " << matches.size() << " inliers size: " << inlierMatches.size() << std::endl;
//
//	double inlierRatio = RANSAC::pointInlierRatio(inlierMatches, matches);
//
////	if ( inlierRatio > )
////	showMatches(sensorFrames[0].rgbImage, features[0], sensorFrames[1].rgbImage, features[1],
////								matches);
//
////	std::vector<std::pair<int, int>> pairedFeatures; // dummy
////	return matchPose2Pose(featureSet, pairedFeatures, estimatedTransformation);
//	return inlierRatio;
//}
