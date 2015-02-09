/** @file matcher.cpp
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#include "../include/Matcher/matcher.h"
#include "../include/Matcher/RGBD.h"

using namespace putslam;

void Matcher::loadInitFeatures(const SensorFrame &sensorData)
{

	// Detect salient features
	prevFeatures = detectFeatures(sensorData.rgbImage);

	// Show detected features
	if (matcherParameters.verbose > 1)
		showFeatures(sensorData.rgbImage, prevFeatures);

	// Remove distortion
	std::vector<cv::Point2f> prevFeaturesUndistorted =
			RGBD::removeImageDistortion(prevFeatures, cameraMatrixMat, distortionCoeffsMat);

	// Remove features without depth
	// TODO: We might remove wrong features due to lack of undisort
	//RGBD::removeFeaturesWithoutDepth(prevFeatures, sensorData.depthImage);

	// Describe salient features
	prevDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeatures, sensorData.depthImage,
			cameraMatrixMat, distortionCoeffsMat);

	// Save rgb/depth images
	prevRgbImage = sensorData.rgbImage;
	prevDepthImage = sensorData.depthImage;
}

Matcher::featureSet Matcher::getFeatures()
{
	featureSet returnSet;
	returnSet.descriptors = prevDescriptors;
	returnSet.feature2D = prevFeatures;
	returnSet.feature3D = prevFeatures3D;
	return returnSet;
}


bool Matcher::match(const SensorFrame& sensorData, Eigen::Matrix4f &estimatedTransformation) {
	// Detect salient features
	std::vector<cv::KeyPoint> features = detectFeatures(sensorData.rgbImage);

	if (matcherParameters.verbose > 1)
			showFeatures(sensorData.rgbImage, features);

	// Remove features without depth
	// TODO: We might remove wrong features due to lack of undisort
	//RGBD::removeFeaturesWithoutDepth(features, sensorData.depthImage);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(sensorData.rgbImage, features);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(features,
			sensorData.depthImage, cameraMatrixMat, distortionCoeffsMat);

	// Visualize matches
	if (matcherParameters.verbose > 0)
			showMatches(prevRgbImage, prevFeatures, sensorData.rgbImage, features, matches);

	// RANSAC
	// - neglect inlierMatches if you do not need them
	std::vector<cv::DMatch> inlierMatches;
	RANSAC ransac(matcherParameters.RANSACParams);
	estimatedTransformation = ransac.estimateTransformation(prevFeatures3D, features3D, matches, inlierMatches);

	// Save computed values for next iteration
	features.swap(prevFeatures);
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
std::vector<Eigen::Vector3f> Matcher::extractMapFeaturesPositions(std::vector<MapFeature> mapFeatures) {

	std::vector<Eigen::Vector3f> mapFeaturePositions3D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
	mapFeaturePositions3D.begin(),
	[](const MapFeature& m) { return m.position.vector().cast<float>(); });
	return mapFeaturePositions3D;
	}

bool Matcher::match(std::vector<MapFeature> mapFeatures,
		std::vector<MapFeature> &foundInlierMapFeatures)
{
	// The current pose descriptors are renamed to make it less confusing
	cv::Mat currentPoseDescriptors(prevDescriptors);

	// We need to extract descriptors and positions from vector<class> to independent vectors to use OpenCV functions
	cv::Mat mapDescriptors = extractMapDescriptors(mapFeatures);
	std::vector<Eigen::Vector3f> mapFeaturePositions3D = extractMapFeaturesPositions(mapFeatures);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(mapDescriptors,
			prevDescriptors);

	// RANSAC
	std::vector<cv::DMatch> inlierMatches;
	RANSAC ransac(matcherParameters.RANSACParams);
	Eigen::Matrix4f estimatedTransformation = ransac.estimateTransformation(
			mapFeaturePositions3D, prevFeatures3D, matches, inlierMatches);

	// for all inliers, convert them to map-compatible format
	foundInlierMapFeatures.clear();
	for (std::vector<cv::DMatch>::iterator it = inlierMatches.begin(); it!=inlierMatches.end(); ++it) {
		int mapId = it->queryIdx, currentPoseId = it->trainIdx;

		MapFeature mapFeature;
		mapFeature.id = mapId;
		mapFeature.position = Vec3(prevFeatures3D[currentPoseId].cast<double>());
		// TODO: take into account the future orientation
		ExtendedDescriptor featureExtendedDescriptor(Quaternion::Identity(), prevDescriptors.row(currentPoseId) );
		mapFeature.descriptors.push_back(featureExtendedDescriptor);
	}
}


void Matcher::showFeatures(cv::Mat rgbImage, std::vector<cv::KeyPoint> features)
{
	cv::Mat imageToShow;
	cv::drawKeypoints(rgbImage,prevFeatures,imageToShow);

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
