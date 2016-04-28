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

void Matcher::loadInitFeatures(const SensorFrame &sensorData) {
	// Detect salient features
	prevFeatures = detectFeatures(sensorData.rgbImage);

	if (matcherParameters.verbose > 2)
		std::cout << "Before dbScan: " << prevFeatures.size() << std::endl;

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
	dbscan.run(prevFeatures);

	if (matcherParameters.verbose > 2)
			std::cout << "After dbScan: " << prevFeatures.size() << std::endl;

	// Show detected features
	if (matcherParameters.verbose > 2)
		showFeatures(sensorData.rgbImage, prevFeatures);

	// Describe salient features if needed
	if (matcherParameters.VOVersion == MatcherParameters::VO_MATCHING
			|| matcherParameters.MapMatchingVersion
					!= MatcherParameters::MAPMATCH_PATCHES) {
		prevDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);
	}

	// Extract 2D points from keypoints
	cv::KeyPoint::convert(prevFeatures,prevFeaturesDistorted);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevFeatures,
			matcherParameters.cameraMatrixMat,
			matcherParameters.distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted,
			sensorData.depthImage, matcherParameters.cameraMatrixMat,
			sensorData.depthImageScale);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;
	prevDepthImageScale = sensorData.depthImageScale;
}

Matcher::featureSet Matcher::getFeatures() {
	featureSet returnSet;
	returnSet.descriptors = prevDescriptors;
	returnSet.feature2D = prevFeatures;
	returnSet.undistortedFeature2D = prevFeaturesUndistorted;
	returnSet.feature3D = prevFeatures3D;
	return returnSet;
}

void Matcher::mergeTrackedFeatures(
		std::vector<cv::Point2f>& undistortedFeatures2D,
		const std::vector<cv::Point2f>& featuresSandBoxUndistorted,
		std::vector<cv::Point2f>& distortedFeatures2D,
		const std::vector<cv::Point2f>& featuresSandBoxDistorted) {

	// Merging features - rejecting feature too close to existing ones
	for (int i = 0; i < featuresSandBoxUndistorted.size(); i++) {
		bool addFeature = true;
		for (int j = 0; j < undistortedFeatures2D.size(); j++) {
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
		}
	}

}

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
		return trackKLT(currentSensorFrame, estimatedTransformation,
				inlierMatches);
		// Something unrecognized
	} else {
		std::cout
				<< "Unrecognized VO choice -> double check matcherOpenCVParameters.xml"
				<< std::endl;
		return 0.0;
	}
}

double Matcher::trackKLT(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Current 2D positions, 3D positions and found matches
	//std::vector<cv::Point2f> undistortedFeatures2D(prevFeaturesUndistorted), distortedFeatures2D;
	std::vector<cv::Point2f> undistortedFeatures2D, distortedFeatures2D;
	std::vector<Eigen::Vector3f> features3D;
	std::vector<cv::DMatch> matches;

	// No features so identity()
	if (prevFeaturesUndistorted.size() == 0 || prevFeaturesDistorted.size() == 0) {
		estimatedTransformation = Eigen::Matrix4f::Identity();
	} else {
		// Tracking features and creating potential matches
		matches = performTracking(prevRgbImage, sensorData.rgbImage,
						prevFeaturesDistorted, distortedFeatures2D);

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
			cv::KeyPoint::convert(distortedFeatures2D, prevFeatures);
			showFeatures(sensorData.rgbImage, prevFeatures);
		}

		// Associate depth -> creating 3D features
		features3D = RGBD::keypoints2Dto3D(undistortedFeatures2D,
				sensorData.depthImage, matcherParameters.cameraMatrixMat,
				sensorData.depthImageScale);

		// Remove features based on 2D and 3D distance
		if ( matcherParameters.OpenCVParams.removeTooCloseFeatures > 0)
			removeTooCloseFeatures(distortedFeatures2D, undistortedFeatures2D, features3D, matches);

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
	if (undistortedFeatures2D.size()
			< matcherParameters.OpenCVParams.minimalTrackedFeatures) {

		// Detect new salient features
		std::vector<cv::KeyPoint> featuresSandbox = detectFeatures(
				sensorData.rgbImage);

		// DBScan on detected features to remove groups of points
		DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
		dbscan.run(featuresSandbox);

		// Extract 2D points from keypoints
		std::vector<cv::Point2f> featuresSandBoxDistorted;
		cv::KeyPoint::convert(featuresSandbox, featuresSandBoxDistorted);

		// Find 2D positions without distortion
		std::vector<cv::Point2f> featuresSandBoxUndistorted =
				RGBD::removeImageDistortion(featuresSandbox,
						matcherParameters.cameraMatrixMat,
						matcherParameters.distortionCoeffsMat);

		// Merging features - rejecting feature too close to existing ones
		// Parameters: (existing features and vector to add new features, new features, minimal image distance between features)
        //std::cout<<"We had " << distortedFeatures2D.size() << std::endl;
		mergeTrackedFeatures(undistortedFeatures2D, featuresSandBoxUndistorted, distortedFeatures2D, featuresSandBoxDistorted);
        //std::cout<<"After merging " << distortedFeatures2D.size() << std::endl;

		// Add depth to new features
		std::vector<Eigen::Vector3f> newFeatures3D = RGBD::keypoints2Dto3D(
				undistortedFeatures2D, sensorData.depthImage,
				matcherParameters.cameraMatrixMat, sensorData.depthImageScale,
				features3D.size());

		// Merge 3D positions of old and new features
		features3D.reserve(features3D.size() + newFeatures3D.size());
		features3D.insert(features3D.end(), newFeatures3D.begin(),
				newFeatures3D.end());

		// Remove features based on 2D and 3D distance
		if (matcherParameters.OpenCVParams.removeTooCloseFeatures > 0)
			removeTooCloseFeatures(distortedFeatures2D, undistortedFeatures2D,
					features3D, matches);

	}

	// In case we need descriptors
	if (matcherParameters.MapMatchingVersion
			!= MatcherParameters::MAPMATCH_PATCHES) {

		// Converting features to keypoints
		std::vector<cv::KeyPoint> prevKeypoints;
		cv::KeyPoint::convert(undistortedFeatures2D, prevKeypoints);

		// Computing descriptors
		prevDescriptors = describeFeatures(sensorData.rgbImage, prevKeypoints);


		// Some unlucky case --> couldn't describe a feature, so we need to remove it and recompute 3D positions
		if (prevKeypoints.size() != undistortedFeatures2D.size()) {

			std::vector<cv::KeyPoint> allKeypoints;
			cv::KeyPoint::convert(undistortedFeatures2D, allKeypoints);

			std::vector<cv::Point2f> tmpDistorted, tmpUndistorted;
			std::vector<Eigen::Vector3f> tmp3D;

			for(int i=0, j=0;i<allKeypoints.size();i++) {
				if ( j == prevKeypoints.size())
					break;
				if (cv::norm(allKeypoints[i].pt - prevKeypoints[j].pt) < 0.0001)
				{
					tmpDistorted.push_back(distortedFeatures2D[i]);
					tmpUndistorted.push_back(undistortedFeatures2D[i]);
					tmp3D.push_back(features3D[i]);
					j++;
				}
			}
			tmpDistorted.swap(distortedFeatures2D);
			tmpUndistorted.swap(undistortedFeatures2D);
			tmp3D.swap(features3D);
		}
	}

	// Check that the sizes are ok
	assert(
			("TrackKLT: 2D and 3D sizes at the end", undistortedFeatures2D.size()
					== features3D.size()));
	assert(
			("TrackKLT: 2D and 3D sizes at the end 2", undistortedFeatures2D.size()
					== prevDescriptors.rows));

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

	// Save computed values for next iteration
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	distortedFeatures2D.swap(prevFeaturesDistorted);
	features3D.swap(prevFeatures3D);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;


	return inlierRatio;
}

double Matcher::match(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Detect salient features
//	auto start = std::chrono::high_resolution_clock::now();
	std::vector<cv::KeyPoint> features = detectFeatures(sensorData.rgbImage);
//	auto duration = std::chrono::duration_cast < std::chrono::microseconds
//			> (std::chrono::high_resolution_clock::now() - start);
//	std::cout << "---->Time:\t Detection time: " << duration.count() / 1000.0
//			<< " ms" << std::endl;

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
//	start = std::chrono::high_resolution_clock::now();
	dbscan.run(features);
//	duration = std::chrono::duration_cast < std::chrono::microseconds
//			> (std::chrono::high_resolution_clock::now() - start);
//	std::cout << "---->Time:\t DBSCAN: " << duration.count() / 1000.0 << " ms"
//			<< std::endl;

	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, features);

	// Describe salient features
//	start = std::chrono::high_resolution_clock::now();
	cv::Mat descriptors = describeFeatures(sensorData.rgbImage, features);
//	duration = std::chrono::duration_cast < std::chrono::microseconds
//			> (std::chrono::high_resolution_clock::now() - start);
//	std::cout << "---->Time:\t Description: " << duration.count() / 1000.0
//			<< " ms" << std::endl;

	// Perform matching
//	start = std::chrono::high_resolution_clock::now();
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);
//	duration = std::chrono::duration_cast < std::chrono::microseconds
//			> (std::chrono::high_resolution_clock::now() - start);
//	std::cout << "---->Time:\t Matching: " << duration.count() / 1000.0 << " ms"
//			<< std::endl;

	// Find 2D positions without distortion
//	start = std::chrono::high_resolution_clock::now();
	std::vector<cv::Point2f> undistortedFeatures2D =
			RGBD::removeImageDistortion(features,
					matcherParameters.cameraMatrixMat,
					matcherParameters.distortionCoeffsMat);
//	duration = std::chrono::duration_cast < std::chrono::microseconds
//			> (std::chrono::high_resolution_clock::now() - start);
//	std::cout << "---->Time:\t undistortion: " << duration.count() / 1000.0
//			<< " ms" << std::endl;

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(
			undistortedFeatures2D, sensorData.depthImage,
			matcherParameters.cameraMatrixMat, sensorData.depthImageScale);

	// Visualize matches
	if (matcherParameters.verbose > 1)
		showMatches(prevRgbImage, prevFeatures, sensorData.rgbImage, features,
				matches);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	matcherParameters.RANSACParams.errorVersion =
			matcherParameters.RANSACParams.errorVersionVO;
	RANSAC ransac(matcherParameters.RANSACParams,
			matcherParameters.cameraMatrixMat);
//	auto start = std::chrono::high_resolution_clock::now();
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
			features3D, matches, inlierMatches);
//	auto duration = std::chrono::duration_cast < std::chrono::microseconds
//				> (std::chrono::high_resolution_clock::now() - start);
//		std::cout << "---->Time:\t RANSAC: " << duration.count() / 1000.0
//				<< " ms" << std::endl;

	// Check
	assert(("Match sizes", features.size() == undistortedFeatures2D.size()));
	assert(
			("Match sizes 2", undistortedFeatures2D.size() == features3D.size()));
	assert(("Match sizes 3", features3D.size() == descriptors.rows));

	// Save computed values for next iteration
	features.swap(prevFeatures);
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	features3D.swap(prevFeatures3D);
	cv::swap(descriptors, prevDescriptors);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;

	return RANSAC::pointInlierRatio(inlierMatches, matches);
}

// We have chosen to use the first descriptor. TODO: Change it to be based on orientation
cv::Mat Matcher::extractMapDescriptors(std::vector<MapFeature> mapFeatures) {
	cv::Mat mapDescriptors;
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it) {
		mapDescriptors.push_back(it->descriptors[0].descriptor);
	}
	return mapDescriptors;
}

// Again, need to extract the Vec3 position of feature to reasonable format -> using nice std::algorithm
std::vector<Eigen::Vector3f> Matcher::extractMapFeaturesPositions(
		std::vector<MapFeature> mapFeatures) {

	std::vector<Eigen::Vector3f> mapFeaturePositions3D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
			mapFeaturePositions3D.begin(),
			[](const MapFeature& m) {return m.position.vector().cast<float>();});
	return mapFeaturePositions3D;
}

double Matcher::match(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation) {
	// The current pose descriptors are renamed to make it less confusing
	cv::Mat currentPoseDescriptors(prevDescriptors);

	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
	cv::Mat mapDescriptors = extractMapDescriptors(mapFeatures);
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(mapDescriptors,
			prevDescriptors);

	// No matches -> impossible to estimate transformation
	if (matches.size() <= 0)
		return -1.0;

	// Choosing version of RANSAC
	std::vector<cv::DMatch> inlierMatches;
	matcherParameters.RANSACParams.errorVersion =
			matcherParameters.RANSACParams.errorVersionMap;

	// Creating and estimating transformation
	RANSAC ransac(matcherParameters.RANSACParams,
			matcherParameters.cameraMatrixMat);
	estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, prevFeatures3D, matches, inlierMatches);

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
				prevFeatures3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);

		ExtendedDescriptor featureExtendedDescriptor(sensorPoseId,
				prevDescriptors.row(currentPoseId));
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}

	return RANSAC::pointInlierRatio(inlierMatches, matches);
}

void Matcher::framesIds2framesIndex(std::vector<MapFeature> featureSet,
		std::vector<int> frameIds, std::vector<int> &closestFrameIndex) {
	int j = 0;
	for (std::vector<MapFeature>::iterator it = featureSet.begin();
			it != featureSet.end(); ++it, ++j) {
		// Find the closest view in a map for a feature
		if (frameIds.size() > 0) {
			for (int k = 0; k < it->descriptors.size(); k++) {
				if (frameIds[j] == it->descriptors[k].poseId) {
					closestFrameIndex[j] = k;
					break;
				}
			}
		}
	}
}

/// Run the match with two poses from the map. Parameters:
/// 	featureSet[2] 	-> set of features from the first and the second pose.
///			Remark: It only makes sense when features are moved to the local
///					coordinate system of features before calling the function
///		frameIds[2] 	-> set of ids of poses for features with the closest angle of observations -
///					in other words for each descriptor, we get the id of the pose so we can take the apprioprate descriptor
/// 	pairedFeatures 	-> return pairs of ids of features that are matched
///		estimatedTransformation		-> the estimated transformation
double Matcher::matchPose2Pose(std::vector<MapFeature> featureSet[2],
		std::vector<int> frameIds[2],
		std::vector<std::pair<int, int>> &pairedFeatures,
		Eigen::Matrix4f &estimatedTransformation) {


	int normType = cv::NORM_HAMMING;
	if (matcherParameters.OpenCVParams.descriptor == "SURF"
				|| matcherParameters.OpenCVParams.descriptor == "SIFT")
		normType = cv::NORM_L2;

	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
    cv::Mat descriptors[2] = { extractMapDescriptors(featureSet[0]),
    		extractMapDescriptors(featureSet[1]) };

	std::vector<Eigen::Vector3f> featurePositions3D[2] = {
			extractMapFeaturesPositions(featureSet[0]),
			extractMapFeaturesPositions(featureSet[1]) };

	std::vector<cv::DMatch> matches = performMatching(descriptors[0], descriptors[1]);


    if (matcherParameters.verbose > 0)
		std::cout << "MatchPose2Pose - we found : " << matches.size() << std::endl;


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
			featurePositions3D[0], featurePositions3D[1], matches,
			inlierMatches);

	return RANSAC::pointInlierRatio(inlierMatches, matches);
}

/// Run the match with two poses from the map. Parameters:
/// 	featureSet[2] 	-> set of features from the first and the second pose.
///			Remark: It only makes sense when features are moved to the local
///					coordinate system of features before calling the function
/// 	pairedFeatures 	-> return pairs of ids of features that are matched
///		estimatedTransformation		-> the estimated transformation
double Matcher::matchPose2Pose(std::vector<MapFeature> featureSet[2],
		std::vector<std::pair<int, int>> &pairedFeatures,
		Eigen::Matrix4f &estimatedTransformation) {

	// Place to store the poseIds of the chosen descriptors
	std::vector<int> frameIds[2];

	// For both poses
	for (int i = 0; i < 2; i++) {

		// We resize the vectors to have the same size as sets of features
		frameIds[i].resize(featureSet[i].size(), 0);

		// For every feature, we save the original pose id (the id of the first descriptor)
        int j = 0;
		for (std::vector<MapFeature>::iterator it = featureSet[i].begin();
                it != featureSet[i].end(); ++it, ++j) {
			frameIds[i][j] = it->descriptors[0].poseId;
        }
	}

    // Now, we can all the more complex version
    return matchPose2Pose(featureSet, frameIds, pairedFeatures, estimatedTransformation);
}

// Versions as above, but works on two sensor frames (it performs detection on its own)
double Matcher::matchPose2Pose(SensorFrame sensorFrames[2],
		Eigen::Matrix4f &estimatedTransformation) {

	cv::Mat wtf[2];
	std::vector<Eigen::Vector3f> features3D[2];
	std::vector<cv::KeyPoint> features[2];

	std::vector<MapFeature> featureSet[2];
	for (int i = 0; i < 2; i++) {
		// Detect salient features
		features[i] = detectFeatures(
				sensorFrames[i].rgbImage);

//		// DBScan
//		DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
//		dbscan.run(features);

		// Describe salient features
		cv::Mat descriptors = describeFeatures(sensorFrames[i].rgbImage, features[i]);

		// Find 2D positions without distortion
		std::vector<cv::Point2f> undistortedFeatures2D =
				RGBD::removeImageDistortion(features[i],
						matcherParameters.cameraMatrixMat,
						matcherParameters.distortionCoeffsMat);

		// Associate depth
		features3D[i] = RGBD::keypoints2Dto3D(undistortedFeatures2D,
				sensorFrames[i].depthImage, matcherParameters.cameraMatrixMat,
				sensorFrames[i].depthImageScale);

		// Convert to map format
		for (int j=0;j<features3D[i].size();j++) {
			MapFeature mapFeature;
			mapFeature.id = j;

			mapFeature.u = undistortedFeatures2D[j].x;
			mapFeature.v = undistortedFeatures2D[j].y;

			mapFeature.position = Vec3(
					features3D[i][j].cast<double>());
			mapFeature.posesIds.push_back(i);

			ExtendedDescriptor featureExtendedDescriptor(i,
					descriptors.row(j));
			mapFeature.descriptors.push_back(featureExtendedDescriptor);

			// Add the measurement
			featureSet[i].push_back(mapFeature);
        }

		wtf[i] = descriptors;
	}

	std::vector<cv::DMatch> matches = performMatching(wtf[0], wtf[1]), inlierMatches;

//	showMatches(sensorFrames[0].rgbImage, features[0], sensorFrames[1].rgbImage, features[1],
//							matches);

	matcherParameters.RANSACParams.errorVersion =
				matcherParameters.RANSACParams.errorVersionVO;
	RANSAC ransac(matcherParameters.RANSACParams,
				matcherParameters.cameraMatrixMat);

	estimatedTransformation = ransac.estimateTransformation(features3D[0], features3D[1], matches, inlierMatches);

	//std::cout<<"WHY MATCHING IS IRRITATING : " << matches.size() << " inliers size: " << inlierMatches.size() << std::endl;

	double inlierRatio = RANSAC::pointInlierRatio(inlierMatches, matches);

//	if ( inlierRatio > )
//	showMatches(sensorFrames[0].rgbImage, features[0], sensorFrames[1].rgbImage, features[1],
//								matches);

//	std::vector<std::pair<int, int>> pairedFeatures; // dummy
//	return matchPose2Pose(featureSet, pairedFeatures, estimatedTransformation);
	return inlierRatio;
}


double Matcher::matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation, bool newDetection,
		std::vector<int> frameIds) {

	if (!newDetection)
		return matchXYZ(mapFeatures, sensorPoseId, foundInlierMapFeatures,
				estimatedTransformation, prevDescriptors, prevFeatures3D);

	// Detect salient features
	prevFeatures = detectFeatures(prevRgbImage);

	// DBScan
	DBScan dbscan(matcherParameters.OpenCVParams.DBScanEps);
	dbscan.run(prevFeatures);

	// Show detected features
	if (matcherParameters.verbose > 2)
		showFeatures(prevRgbImage, prevFeatures);

	prevDescriptors = describeFeatures(prevRgbImage, prevFeatures);

	// Extract 2D points from keypoints
	cv::KeyPoint::convert(prevFeatures, prevFeaturesDistorted);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevFeatures,
			matcherParameters.cameraMatrixMat,
			matcherParameters.distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted,
			prevDepthImage, matcherParameters.cameraMatrixMat,
			prevDepthImageScale);

	return matchXYZ(mapFeatures, sensorPoseId, foundInlierMapFeatures,
			estimatedTransformation, prevDescriptors, prevFeatures3D);
}



double Matcher::matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation,
		cv::Mat currentPoseDescriptors,
		std::vector<Eigen::Vector3f> &currentPoseFeatures3D,std::vector<int> frameIds) {

	double matchingXYZSphereRadius = 0.15;
	double matchingXYZacceptRatioOfBestMatch = 0.85;

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



	// Perform matching
	std::vector<cv::DMatch> matches;

	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
	cv::Mat mapDescriptors = extractMapDescriptors(mapFeatures);
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// For all features in the map
	int j = 0, perfectMatchCounter = 0;
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it, ++j) {

		// Find the closest view in a map for a feature
		int mapFeatureClosestFrameId = 0;
		if (frameIds.size() > 0) {
			for (int k = 0; k < it->descriptors.size(); k++) {
				if (frameIds[j] == it->descriptors[k].poseId) {
					mapFeatureClosestFrameId = k;
					break;
				}
			}
		}

		// Possible matches for considered feature
		std::vector<int> possibleMatchId;

		// Reject all matches that are further away than threshold
		for (int i = 0; i < currentPoseFeatures3D.size(); i++) {
			Eigen::Vector3f tmp((float) it->position.x(),
					(float) it->position.y(), (float) it->position.z());
			float norm = (tmp - (currentPoseFeatures3D[i])).norm();

			if (norm < matchingXYZSphereRadius) {
				possibleMatchId.push_back(i);
			}
		}

		// Find best match based on descriptors
		int bestId = -1;
		float bestVal;
		for (int i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x = (it->descriptors[mapFeatureClosestFrameId].descriptor
					- currentPoseDescriptors.row(id));
			float value = norm(x, normType);
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
		for (int i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x = (it->descriptors[mapFeatureClosestFrameId].descriptor
					- currentPoseDescriptors.row(id));
			float value = norm(x, normType);
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

		ExtendedDescriptor featureExtendedDescriptor(sensorPoseId,
				currentPoseDescriptors.row(currentPoseId));
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}

	// Compute inlier ratio
	return RANSAC::pointInlierRatio(inlierMatches, matches);
}

double Matcher::matchToMapUsingPatches(std::vector<MapFeature> mapFeatures,
		int sensorPoseId, putslam::Mat34 cameraPose, std::vector<int> frameIds,
		std::vector<putslam::Mat34> cameraPoses,
		std::vector<cv::Mat> mapRgbImages, std::vector<cv::Mat> mapDepthImages,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation, double depthImageScale,
		std::vector<std::pair<double, double>> &errorLog,
		bool withRANSAC) {

	// Create matching on patches with wanted params
	MatchingOnPatches matchingOnPatches(matcherParameters.PatchesParams);

	// Optimize patch locations
	std::vector<cv::Point2f> optimizedLocations;
	std::vector<cv::DMatch> matches;

	// For all features
	for (int i = 0, goodFeaturesIndex = 0; i < mapFeatures.size(); i++) {

		// Compute the 4 points/borders of the new patch (Clockwise)
		std::vector<cv::Point2f> warpingPoints;
		const int halfPatchSize = matchingOnPatches.getHalfPatchSize();
		const int halfPatchBorderSize = halfPatchSize + 1;
		const int patchSize = matchingOnPatches.getPatchSize();
		const int patchBorderSize = patchSize + 2;

		// Saving those points in OpenCV types
		warpingPoints.push_back(
				cv::Point2f(mapFeatures[i].u - halfPatchBorderSize,
						mapFeatures[i].v - halfPatchBorderSize));
		warpingPoints.push_back(
				cv::Point2f(mapFeatures[i].u + halfPatchBorderSize,
						mapFeatures[i].v - halfPatchBorderSize));
		warpingPoints.push_back(
				cv::Point2f(mapFeatures[i].u + halfPatchBorderSize,
						mapFeatures[i].v + halfPatchBorderSize));
		warpingPoints.push_back(
				cv::Point2f(mapFeatures[i].u - halfPatchBorderSize,
						mapFeatures[i].v + halfPatchBorderSize));

		// Project those 4 points into space
		std::vector<Eigen::Vector3f> warpingPoints3D = RGBD::keypoints2Dto3D(
				warpingPoints, prevDepthImage,
				matcherParameters.cameraMatrixMat, depthImageScale);

		// Check if all points are correct - reject those without depth as those invalidate the patch planarity assumption
		bool correctPoints3D = true;
		for (int i = 0; i < warpingPoints3D.size(); i++) {
			if (warpingPoints3D[i].z() < 0.0001) {
				correctPoints3D = false;
				break;
			}
		}

		// If those points are correct - we perform matching on patches
		bool featureOK = true;
		if (correctPoints3D) {

			cv::Mat warpedImage;
			double uMap = 0.0, vMap = 0.0;
			if (matcherParameters.PatchesParams.warping) {
				// Move to global coordinate and then to local of original feature detection
				std::vector<cv::Point2f> src(4);
				for (int i = 0; i < 4; i++) {
					putslam::Mat34 warp(
							putslam::Vec3(warpingPoints3D[i].cast<double>()));
					warp = (cameraPoses[i].inverse()).matrix()
							* cameraPose.matrix() * warp.matrix();

					float u = warp(0, 3)
							* matcherParameters.cameraMatrixMat.at<float>(0, 0)
							/ warp(2, 3)
							+ matcherParameters.cameraMatrixMat.at<float>(0, 2);
					float v = warp(1, 3)
							* matcherParameters.cameraMatrixMat.at<float>(1, 1)
							/ warp(2, 3)
							+ matcherParameters.cameraMatrixMat.at<float>(1, 2);
					src[i] = cv::Point2f(u, v);
				}

				// We will transform it into the rectangle
				std::vector<cv::Point2f> dst(4);
				dst[0] = cv::Point2f(0, 0);
				dst[1] = cv::Point2f(patchSize + 1, 0);
				dst[2] = cv::Point2f(patchSize + 1, patchSize + 1);
				dst[3] = cv::Point2f(0, patchSize + 1);

				// Compute getPerspective
				cv::Mat perspectiveTransform = cv::getPerspectiveTransform(src,
						dst);

				// Compute warpPerspective
				cv::warpPerspective(mapRgbImages[i], warpedImage,
						perspectiveTransform,
						cv::Size(patchBorderSize, patchBorderSize));

				// Position to compute the map patch
				uMap = halfPatchSize + 1;
				vMap = halfPatchSize + 1;
			} else {
				// Find positions on original image
				float uMap = -1, vMap = -1;
				for (int j = 0; j < mapFeatures[i].descriptors.size(); j++) {
					if (mapFeatures[i].descriptors[j].poseId == frameIds[i]) {
						uMap = mapFeatures[i].descriptors[j].u;
						vMap = mapFeatures[i].descriptors[j].v;
						break;
					}
				}

				// Set image as:
				warpedImage = mapRgbImages[i];
			}

			// Compute old patch
			std::vector<double> patchMap;
			patchMap = matchingOnPatches.computePatch(warpedImage, uMap, vMap);

			// Compute gradient
			std::vector<float> gradientX, gradientY;
			Eigen::Matrix3f InvHessian = Eigen::Matrix3f::Zero();
			matchingOnPatches.computeGradient(warpedImage, uMap, vMap,
					InvHessian, gradientX, gradientY);

			// Print information
			if (matcherParameters.verbose > 0)
				std::cout << "Patches preoptimization: " << mapFeatures[i].u
						<< " " << mapFeatures[i].v << std::endl;

			// Optimize position of the feature
			float_type uOld = mapFeatures[i].u;
			float_type vOld = mapFeatures[i].v;
			featureOK = matchingOnPatches.optimizeLocation(warpedImage,
					patchMap, prevRgbImage, mapFeatures[i].u, mapFeatures[i].v,
					gradientX, gradientY, InvHessian);

			// Save ok
			if (featureOK) {
				double error2D = std::sqrt(
						(mapFeatures[i].u - uOld) * (mapFeatures[i].u - uOld)
								+ (mapFeatures[i].v - vOld)
										* (mapFeatures[i].v - vOld));
				std::cout << "Patches 2D diff: " << error2D << std::endl;
				Eigen::Vector3f p3D = RGBD::point2Dto3D(
						cv::Point2f(mapFeatures[i].u, mapFeatures[i].v),
						prevDepthImage, matcherParameters.cameraMatrixMat,
						depthImageScale);
				Eigen::Vector3f r3D = RGBD::point2Dto3D(cv::Point2f(uOld, vOld),
						prevDepthImage, matcherParameters.cameraMatrixMat,
						depthImageScale);
				double error3D = (r3D - p3D).norm();
				std::cout << "Patches 3D diff: " << error3D << std::endl;

				errorLog.push_back(std::make_pair(error2D, error3D));
			}

			if (!featureOK) {
				mapFeatures[i].u = uOld;
				mapFeatures[i].v = vOld;
				featureOK = true;
			}

			// Print information
			if (matcherParameters.verbose > 0)
				std::cout << "Patches: " << featureOK << " " << mapFeatures[i].u
						<< " " << mapFeatures[i].v << std::endl;
		}
		// Save a good match
		if (featureOK || !correctPoints3D) {
			matches.push_back(cv::DMatch(i, goodFeaturesIndex, 0));
			optimizedLocations.push_back(
					cv::Point2f(mapFeatures[i].u, mapFeatures[i].v));
			goodFeaturesIndex++;
		}

	}

	// Project optimized features into 3D features
	std::vector<Eigen::Vector3f> optimizedMapLocations3D =
			RGBD::keypoints2Dto3D(optimizedLocations, prevDepthImage,
					matcherParameters.cameraMatrixMat, depthImageScale);

	// Extract 3D from map
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// RANSAC part
	if (matcherParameters.verbose > 0)
		std::cout << "Matches on patches counter: " << matches.size()
				<< std::endl;

	if (matches.size() <= 0)
		return -1.0;

	std::vector<cv::DMatch> inlierMatches2;

	// should we use RANSAC?
	if (withRANSAC) {
		matcherParameters.RANSACParams.errorVersion =
				matcherParameters.RANSACParams.errorVersionMap;
		RANSAC ransac(matcherParameters.RANSACParams,
				matcherParameters.cameraMatrixMat);
		estimatedTransformation = ransac.estimateTransformation(
				mapFeaturePositions3D, optimizedMapLocations3D, matches,
				inlierMatches2);
	} else
		inlierMatches2 = matches;

	// for all inliers, convert them to map-compatible format
	foundInlierMapFeatures.clear();
	for (std::vector<cv::DMatch>::iterator it = inlierMatches2.begin();
			it != inlierMatches2.end(); ++it) {
		int mapId = it->queryIdx, currentPoseId = it->trainIdx;

		MapFeature mapFeature;
		mapFeature.id = mapFeatures[mapId].id;

		mapFeature.u = optimizedLocations[currentPoseId].x;
		mapFeature.v = optimizedLocations[currentPoseId].y;

		mapFeature.position = Vec3(
				optimizedMapLocations3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);

		ExtendedDescriptor featureExtendedDescriptor(sensorPoseId, mapFeature.u,
				mapFeature.v, cv::Mat());
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}

	return RANSAC::pointInlierRatio(inlierMatches2, matches);
}

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
		std::vector<Eigen::Vector3f> &features3D, std::vector<cv::DMatch> &matches){

	// Check that we have the same sizes
	assert(("removeTooCloseFeatures: distorted vs undistorted sizes", undistortedFeatures2D.size()
							== distortedFeatures2D.size()));
	assert(("removeTooCloseFeatures: features3D vs undistorted sizes", features3D.size()
								== distortedFeatures2D.size()));

	std::set<int> featuresToRemove;

	for (int i = 0; i < features3D.size(); i++) {
		for (int j = i + 1; j < features3D.size(); j++) {

			double x = features3D[i][0] - features3D[j][0];
			double y = features3D[i][1] - features3D[j][1];
			double z = features3D[i][2] - features3D[j][2];
			double dist3D = sqrt(x * x + y * y + z * z);

			double u = undistortedFeatures2D[i].x - undistortedFeatures2D[j].x;
			double v = undistortedFeatures2D[i].y - undistortedFeatures2D[j].y;
			double dist2D = sqrt(u * u + v * v);

			if ( dist3D < matcherParameters.OpenCVParams.minimalEuclidDistanceNewTrackingFeatures ||
					dist2D < matcherParameters.OpenCVParams.minimalReprojDistanceNewTrackingFeatures) {
				featuresToRemove.insert(j); // TODO: Arbitrary decision right now
			}
		}
	}

	// Removing from distorted Features
	int count = 0;
	distortedFeatures2D.erase(
			std::remove_if(distortedFeatures2D.begin(),
					distortedFeatures2D.end(),
					[&](const cv::Point2f & o) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			distortedFeatures2D.end());

	// Removing from undistorted Features
	count = 0;
	undistortedFeatures2D.erase(
			std::remove_if(undistortedFeatures2D.begin(),
					undistortedFeatures2D.end(),
					[&](const cv::Point2f & o) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			undistortedFeatures2D.end());

	// Removing from features 3D
	count = 0;
	features3D.erase(
			std::remove_if(features3D.begin(), features3D.end(),
					[&](const Eigen::Vector3f & o) {return featuresToRemove.find(count++) != featuresToRemove.end();}),
			features3D.end());

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
	return prevFeaturesDistorted.size();
}
