/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/Matcher/RGBD.h"
#include "../include/Matcher/dbscan.h"

#include <chrono>

using namespace putslam;

void Matcher::loadInitFeatures(const SensorFrame &sensorData) {
	// Detect salient features
	prevFeatures = detectFeatures(sensorData.rgbImage);

	// DBScan
	DBScan dbscan(8, 2, 1);
	dbscan.run(prevFeatures);

	// Show detected features
	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, prevFeatures);

	// Describe salient features
	prevDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevFeatures,
			matcherParameters.cameraMatrixMat,
			matcherParameters.distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted,
			sensorData.depthImage, matcherParameters.cameraMatrixMat);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;
}

Matcher::featureSet Matcher::getFeatures() {
	featureSet returnSet;
	returnSet.descriptors = prevDescriptors;
	returnSet.feature2D = prevFeatures;
	returnSet.undistortedFeature2D = prevFeaturesUndistorted;
	returnSet.feature3D = prevFeatures3D;
	return returnSet;
}

bool Matcher::match(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	// Detect salient features
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<cv::KeyPoint> features = detectFeatures(sensorData.rgbImage);
	auto duration = std::chrono::duration_cast < std::chrono::microseconds
			> (std::chrono::high_resolution_clock::now() - start);
	std::cout << "---->Time:\t Detection time: " << duration.count() / 1000.0
			<< " ms" << std::endl;


	// DBScan
	DBScan dbscan(8, 2, 1);
	start = std::chrono::high_resolution_clock::now();
	dbscan.run(features);
	duration = std::chrono::duration_cast < std::chrono::microseconds
			> (std::chrono::high_resolution_clock::now() - start);
	std::cout << "---->Time:\t DBSCAN: " << duration.count() / 1000.0 << " ms"
			<< std::endl;


	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, features);

	// Describe salient features
	start = std::chrono::high_resolution_clock::now();
	cv::Mat descriptors = describeFeatures(sensorData.rgbImage, features);
	duration = std::chrono::duration_cast < std::chrono::microseconds
			> (std::chrono::high_resolution_clock::now() - start);
	std::cout << "---->Time:\t Description: " << duration.count() / 1000.0
			<< " ms" << std::endl;


	// Perform matching
	start = std::chrono::high_resolution_clock::now();
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);
	duration = std::chrono::duration_cast < std::chrono::microseconds
			> (std::chrono::high_resolution_clock::now() - start);
	std::cout << "---->Time:\t Matching: " << duration.count() / 1000.0 << " ms"
			<< std::endl;


	// Find 2D positions without distortion
	start = std::chrono::high_resolution_clock::now();
	std::vector<cv::Point2f> undistortedFeatures2D =
			RGBD::removeImageDistortion(features,
					matcherParameters.cameraMatrixMat,
					matcherParameters.distortionCoeffsMat);
	duration = std::chrono::duration_cast < std::chrono::microseconds
			> (std::chrono::high_resolution_clock::now() - start);
	std::cout << "---->Time:\t undistortion: " << duration.count() / 1000.0
			<< " ms" << std::endl;


	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(
			undistortedFeatures2D, sensorData.depthImage,
			matcherParameters.cameraMatrixMat);

	// Visualize matches
	if (matcherParameters.verbose > 0)
		showMatches(prevRgbImage, prevFeatures, sensorData.rgbImage, features,
				matches);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	RANSAC ransac(matcherParameters.RANSACParams);
	start = std::chrono::high_resolution_clock::now();
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
			features3D, matches, inlierMatches);
	duration = std::chrono::duration_cast < std::chrono::microseconds
				> (std::chrono::high_resolution_clock::now() - start);
		std::cout << "---->Time:\t RANSAC: " << duration.count() / 1000.0
				<< " ms" << std::endl;

	// Save computed values for next iteration
	features.swap(prevFeatures);
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	features3D.swap(prevFeatures3D);
	cv::swap(descriptors, prevDescriptors);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;

	return false;
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

bool Matcher::match(std::vector<MapFeature> mapFeatures, int sensorPoseId,
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

	std::cout << "Matching to map: descriptors - " << mapDescriptors.rows << " "
			<< mapDescriptors.cols << " " << prevDescriptors.rows << " "
			<< prevDescriptors.cols << std::endl;

	std::cout << "Matching to map: feature3D positions - "
			<< mapFeaturePositions3D.size() << " " << prevFeatures3D.size()
			<< std::endl;

	// RANSAC
	std::vector<cv::DMatch> inlierMatches;
	RANSAC ransac(matcherParameters.RANSACParams);
	estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, prevFeatures3D, matches, inlierMatches);

	// for all inliers, convert them to map-compatible format
	foundInlierMapFeatures.clear();
	for (std::vector<cv::DMatch>::iterator it = inlierMatches.begin();
			it != inlierMatches.end(); ++it) {
		int mapId = it->queryIdx, currentPoseId = it->trainIdx;

		MapFeature mapFeature;
		mapFeature.id = mapFeatures[mapId].id;


		// TODO: Test corrections after Dominic question
		mapFeature.u = prevFeaturesUndistorted[currentPoseId].x;
		mapFeature.v = prevFeaturesUndistorted[currentPoseId].y;

		mapFeature.position = Vec3(
				prevFeatures3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);
		// TODO: take into account the future orientation
		ExtendedDescriptor featureExtendedDescriptor(sensorPoseId,
				prevDescriptors.row(currentPoseId));
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}
}

bool Matcher::matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation) {

	// The current pose descriptors are renamed to make it less confusing
	cv::Mat currentPoseDescriptors(prevDescriptors);

	// Perform matching
	std::vector<cv::DMatch> matches;

	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
	cv::Mat mapDescriptors = extractMapDescriptors(mapFeatures);
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// For all features in the map
	int j = 0;
	for (std::vector<MapFeature>::iterator it = mapFeatures.begin();
			it != mapFeatures.end(); ++it, ++j) {

		// Possible matches for considered feature
		std::vector<int> possibleMatchId;

		for (int i = 0; i < prevFeatures3D.size(); i++) {
			Eigen::Vector3f tmp((float) it->position.x(),
					(float) it->position.y(), (float) it->position.z());
			float norm = (tmp - (prevFeatures3D[i])).norm();

			if (norm < 0.10) {
				possibleMatchId.push_back(i);
			}
		}

		int bestId = -1;
		float bestVal = 9999;
		for (int i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x =
					(it->descriptors[0].descriptor - prevDescriptors.row(id));
			float value = norm(x, cv::NORM_L2);
			if (value < bestVal) {
				bestVal = value;
				bestId = id;
			}
		}

		for (int i = 0; i < possibleMatchId.size(); i++) {
			int id = possibleMatchId[i];

			cv::Mat x =
					(it->descriptors[0].descriptor - prevDescriptors.row(id));
			float value = norm(x, cv::NORM_L2);
			if (0.7 * value < bestVal) {
				cv::DMatch tmpMatch;
				tmpMatch.distance = value;
				tmpMatch.queryIdx = j;
				tmpMatch.trainIdx = id;
				matches.push_back(tmpMatch);
			}
		}

//		if ( bestId != -1) {
////			std::cout<<"BEST VALUE : " << bestVal << std::endl;
//			cv::DMatch tmpMatch;
//			tmpMatch.distance = bestVal;
//			tmpMatch.queryIdx = j;
//			tmpMatch.trainIdx = bestId;
//			matches.push_back(tmpMatch);
//		}
	}

	std::cout << "MatchesXYZ - we found : " << matches.size() << std::endl;

	// RANSAC
	std::vector<cv::DMatch> inlierMatches;
	RANSAC ransac(matcherParameters.RANSACParams);
	estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, prevFeatures3D, matches, inlierMatches);

	// for all inliers, convert them to map-compatible format
	foundInlierMapFeatures.clear();
	for (std::vector<cv::DMatch>::iterator it = inlierMatches.begin();
			it != inlierMatches.end(); ++it) {
		int mapId = it->queryIdx, currentPoseId = it->trainIdx;

		MapFeature mapFeature;
		mapFeature.id = mapFeatures[mapId].id;

		// TODO: Test corrections after Dominic question
		mapFeature.u = prevFeaturesUndistorted[currentPoseId].x;
		mapFeature.v = prevFeaturesUndistorted[currentPoseId].y;


		mapFeature.position = Vec3(
				prevFeatures3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);
		// TODO: take into account the future orientation
		ExtendedDescriptor featureExtendedDescriptor(sensorPoseId,
				prevDescriptors.row(currentPoseId));
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}
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
