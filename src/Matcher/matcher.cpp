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

	// Describe salient features
	prevDescriptors = describeFeatures(sensorData.rgbImage, prevFeatures);

	// Remove distortion
	prevFeaturesUndistorted = RGBD::removeImageDistortion(prevFeatures,
			cameraMatrixMat, distortionCoeffsMat);

	// Associate depth
	prevFeatures3D = RGBD::keypoints2Dto3D(prevFeaturesUndistorted, sensorData.depthImage,
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
	returnSet.undistortedFeature2D = prevFeaturesUndistorted;
	returnSet.feature3D = prevFeatures3D;
	return returnSet;
}


bool Matcher::match(const SensorFrame& sensorData, Eigen::Matrix4f &estimatedTransformation) {
	// Detect salient features
	std::vector<cv::KeyPoint> features = detectFeatures(sensorData.rgbImage);

	if (matcherParameters.verbose > 1)
			showFeatures(sensorData.rgbImage, features);

	// Describe salient features
	cv::Mat descriptors = describeFeatures(sensorData.rgbImage, features);

	// Perform matching
	std::vector<cv::DMatch> matches = performMatching(prevDescriptors,
			descriptors);

	// Find 2D positions without distortion
	std::vector<cv::Point2f> undistortedFeatures2D = RGBD::removeImageDistortion(
			features, cameraMatrixMat, distortionCoeffsMat);

	// Associate depth
	std::vector<Eigen::Vector3f> features3D = RGBD::keypoints2Dto3D(undistortedFeatures2D,
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
std::vector<Eigen::Vector3f> Matcher::extractMapFeaturesPositions(std::vector<MapFeature> mapFeatures) {

	std::vector<Eigen::Vector3f> mapFeaturePositions3D(mapFeatures.size());
	std::transform(mapFeatures.begin(), mapFeatures.end(),
	mapFeaturePositions3D.begin(),
	[](const MapFeature& m) { return m.position.vector().cast<float>(); });
	return mapFeaturePositions3D;
	}

bool Matcher::match(std::vector<MapFeature> mapFeatures, int sensorPoseId,
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

	std::cout << "Matching to map: descriptors - " << mapDescriptors.rows << " "
			<< mapDescriptors.cols << " " << prevDescriptors.rows << " "
			<< prevDescriptors.cols << std::endl;

	std::cout << "Matching to map: feature3D positions - " << mapFeaturePositions3D.size() << " "
			<< prevFeatures3D.size() << std::endl;


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
		mapFeature.id = mapFeatures[mapId].id;
		mapFeature.position = Vec3(prevFeatures3D[currentPoseId].cast<double>());
		mapFeature.posesIds.push_back(sensorPoseId);
		// TODO: take into account the future orientation
        ExtendedDescriptor featureExtendedDescriptor(sensorPoseId, prevDescriptors.row(currentPoseId) );
		mapFeature.descriptors.push_back(featureExtendedDescriptor);

		// Add the measurement
		foundInlierMapFeatures.push_back(mapFeature);
	}
}


void Matcher::showFeatures(cv::Mat rgbImage, std::vector<cv::KeyPoint> featuresToShow)
{
	cv::Mat imageToShow;
	cv::drawKeypoints(rgbImage,featuresToShow,imageToShow);

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
