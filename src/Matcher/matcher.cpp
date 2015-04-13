/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/RGBD/RGBD.h"
#include "../include/Matcher/dbscan.h"

#include <chrono>

using namespace putslam;

void Matcher::loadInitFeatures(const SensorFrame &sensorData) {
	// Detect salient features
	prevFeatures = detectFeatures(sensorData.rgbImage);

	std::cout<<prevFeatures.size()<<std::endl;

	// DBScan
	DBScan dbscan(6, 2, 1);
	dbscan.run(prevFeatures);

	std::cout<<prevFeatures.size()<<std::endl;

	// Show detected features
	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, prevFeatures);

	// Describe salient features if needed
		if (matcherParameters.VOVersion == MatcherParameters::VO_MATCHING
			|| matcherParameters.MapMatchingVersion
					!= MatcherParameters::MAPMATCH_PATCHES) {
		prevDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);
	}

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

void Matcher::mergeTrackedFeatures(
		std::vector<cv::Point2f>& undistortedFeatures2D,
		const std::vector<cv::Point2f>& featuresSandBoxUndistorted,
		float euclideanDistance, cv::Mat& descriptors, cv::Mat &featuresSandboxDescriptors) {
	// Merging features - rejecting feature too close to existing ones
	for (int i = 0; i < featuresSandBoxUndistorted.size(); i++) {
		bool addFeature = true;
		for (int j = 0; j < undistortedFeatures2D.size(); j++) {
			if (cv::norm(
					featuresSandBoxUndistorted[i] - undistortedFeatures2D[j])
					< euclideanDistance) {
				addFeature = false;
				break;
			}
		}
		if (addFeature) {
			undistortedFeatures2D.push_back(featuresSandBoxUndistorted[i]);

			if ( !featuresSandboxDescriptors.empty() )
				descriptors.push_back(featuresSandboxDescriptors.row(i));
		}
	}
}


bool Matcher::runVO(const SensorFrame& currentSensorFrame,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {

	if (matcherParameters.VOVersion == Matcher::MatcherParameters::VO_MATCHING) {
		return match(currentSensorFrame, estimatedTransformation, inlierMatches);
	}
	else if (matcherParameters.VOVersion == Matcher::MatcherParameters::VO_TRACKING){
		return trackKLT(currentSensorFrame, estimatedTransformation, inlierMatches);
	}
	else {
		std::cout << "Unrecognized VO choise -> double check matcherOpenCVParameters.xml" << std::endl;
	}

}

bool Matcher::trackKLT(const SensorFrame& sensorData,
		Eigen::Matrix4f &estimatedTransformation,
		std::vector<cv::DMatch> &inlierMatches) {
	// TODO:
	// - if no motion than skip frame

	// Tracking features and creating potential matches
	std::vector<cv::Point2f> undistortedFeatures2D;
	std::vector<cv::DMatch> matches = performTracking(prevRgbImage,
			sensorData.rgbImage, prevFeaturesUndistorted,
			undistortedFeatures2D);

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(
			undistortedFeatures2D, sensorData.depthImage,
			matcherParameters.cameraMatrixMat);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	RANSAC ransac(matcherParameters.RANSACParams, matcherParameters.cameraMatrixMat);
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
			features3D, matches, inlierMatches);


	// If the number of tracked features falls below certain number, we detect new features are merge them together
	// TODO: Parameters
	int minimalTrackingFeatures = 100;
	float euclideanDistance = 4;
	if (undistortedFeatures2D.size() < minimalTrackingFeatures) {
		// Detect salient features
		std::vector<cv::KeyPoint> featuresSandbox = detectFeatures(
				sensorData.rgbImage);

		// DBScan on detected
		DBScan dbscan(6, 2, 1);
		dbscan.run(featuresSandbox);

		cv::Mat featuresSandboxDescriptors;
		if ( matcherParameters.VOVersion
					!= MatcherParameters::MAPMATCH_PATCHES) {
			featuresSandboxDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);
		}

		// Find 2D positions without distortion
		std::vector<cv::Point2f> featuresSandBoxUndistorted =
				RGBD::removeImageDistortion(featuresSandbox,
						matcherParameters.cameraMatrixMat,
						matcherParameters.distortionCoeffsMat);

		// Merging features - rejecting feature too close to existing ones
		// Parameters: (existing features and vector to add new features, new features, minimal euclidean distance between features)
		mergeTrackedFeatures(undistortedFeatures2D, featuresSandBoxUndistorted,
				euclideanDistance, prevDescriptors, featuresSandboxDescriptors);

		// Add depth to new features
		std::vector<Eigen::Vector3f> newFeatures3D = RGBD::keypoints2Dto3D(
				undistortedFeatures2D, sensorData.depthImage,
				matcherParameters.cameraMatrixMat, features3D.size());

		// Merge 3D positions
		features3D.reserve(features3D.size() + newFeatures3D.size());
		features3D.insert(features3D.end(), newFeatures3D.begin(),
				newFeatures3D.end());
	}

	// Save computed values for next iteration
	undistortedFeatures2D.swap(prevFeaturesUndistorted);
	features3D.swap(prevFeatures3D);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;

}

bool Matcher::match(const SensorFrame& sensorData,
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
	DBScan dbscan(8, 2, 1);
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
			matcherParameters.cameraMatrixMat);

	// Visualize matches
	if (matcherParameters.verbose > 0)
		showMatches(prevRgbImage, prevFeatures, sensorData.rgbImage, features,
				matches);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	RANSAC ransac(matcherParameters.RANSACParams, matcherParameters.cameraMatrixMat);
//	auto start = std::chrono::high_resolution_clock::now();
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D,
			features3D, matches, inlierMatches);
//	auto duration = std::chrono::duration_cast < std::chrono::microseconds
//				> (std::chrono::high_resolution_clock::now() - start);
//		std::cout << "---->Time:\t RANSAC: " << duration.count() / 1000.0
//				<< " ms" << std::endl;



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
	RANSAC ransac(matcherParameters.RANSACParams, matcherParameters.cameraMatrixMat);
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
	RANSAC ransac(matcherParameters.RANSACParams, matcherParameters.cameraMatrixMat);
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

bool Matcher::matchToMapUsingPatches(std::vector<MapFeature> mapFeatures,
		int sensorPoseId, putslam::Mat34 cameraPose, std::vector<int> frameIds,
		std::vector<putslam::Mat34> cameraPoses,
		std::vector<cv::Mat> mapRgbImages, std::vector<cv::Mat> mapDepthImages,
		std::vector<MapFeature> &foundInlierMapFeatures,
		Eigen::Matrix4f &estimatedTransformation) {

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
		std::vector<Eigen::Vector3f> warpingPoints3D =
					RGBD::keypoints2Dto3D(warpingPoints,
							prevDepthImage,
							matcherParameters.cameraMatrixMat);

		// Check if all points are correct - reject those without depth as those invalidate the patch planarity assumption
		bool correctPoints3D = true;
		for (int i=0;i<warpingPoints3D.size(); i++) {
			if (warpingPoints3D[i].z() < 0.0001) {
				correctPoints3D = false;
				break;
			}
		}

		// If those points are correct - we perform matching on patches
		if ( correctPoints3D ) {

			cv::Mat warpedImage;
			double uMap = 0.0, vMap = 0.0;
			if ( matcherParameters.PatchesParams.warping )
			{
				// Move to global coordinate and then to local of original feature detection
				std::vector<cv::Point2f> src(4);
				for (int i=0;i<4;i++)
				{
					putslam::Mat34 warp(
							putslam::Vec3(warpingPoints3D[i].cast<double>()));
					warp = (cameraPoses[i].inverse()).matrix() * cameraPose.matrix()
							* warp.matrix();

					float u = warp(0,3)
							* matcherParameters.cameraMatrixMat.at<float>(0, 0)
							/ warp(2,3)
							+ matcherParameters.cameraMatrixMat.at<float>(0, 2);
					float v = warp(1,3)
							* matcherParameters.cameraMatrixMat.at<float>(1, 1)
							/ warp(2,3)
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
				cv::Mat perspectiveTransform = cv::getPerspectiveTransform(src,dst);

				// Compute warpPerspective
				cv::warpPerspective(mapRgbImages[i], warpedImage, perspectiveTransform,
						cv::Size(patchBorderSize, patchBorderSize));

				// Position to compute the map patch
				uMap = halfPatchSize + 1;
				vMap = halfPatchSize + 1;
			}
			else
			{
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
			std::vector<uint8_t> patchMap;
			patchMap = matchingOnPatches.computePatch(warpedImage, uMap, vMap);

			// Compute gradient
			std::vector<float> gradientX, gradientY;
			Eigen::Matrix3f InvHessian = Eigen::Matrix3f::Zero();
			matchingOnPatches.computeGradient(warpedImage, uMap, vMap,
					InvHessian, gradientX, gradientY);

			// Print information
			if ( matcherParameters.verbose > 0 )
				std::cout << "Patches preoptimization: " << mapFeatures[i].u << " "
						<< mapFeatures[i].v << std::endl;

			// Optimize position of the feature
			bool success = matchingOnPatches.optimizeLocation(warpedImage,
					patchMap, prevRgbImage, mapFeatures[i].u, mapFeatures[i].v,
					gradientX, gradientY, InvHessian);

			// Print information
			if ( matcherParameters.verbose > 0 )
				std::cout << "Patches: " << success << " " << mapFeatures[i].u << " "
						<< mapFeatures[i].v << std::endl;

			// Save a good match
			if (success) {
				matches.push_back(cv::DMatch(i, goodFeaturesIndex, 0));
				optimizedLocations.push_back(
						cv::Point2f(mapFeatures[i].u, mapFeatures[i].v));
				goodFeaturesIndex++;
			}
		}

	}

	// Project optimized features into 3D features
	std::vector<Eigen::Vector3f> optimizedMapLocations3D =
			RGBD::keypoints2Dto3D(optimizedLocations,
					prevDepthImage,
					matcherParameters.cameraMatrixMat);

	// Extract 3D from map
	std::vector<Eigen::Vector3f> mapFeaturePositions3D =
			extractMapFeaturesPositions(mapFeatures);

	// RANSAC
	if ( matcherParameters.verbose > 0 )
		std::cout << "Matches on patches counter: " << matches.size() << std::endl;


	std::vector<cv::DMatch> inlierMatches2;
	RANSAC ransac(matcherParameters.RANSACParams, matcherParameters.cameraMatrixMat);
	estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, optimizedMapLocations3D, matches,
			inlierMatches2);

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
