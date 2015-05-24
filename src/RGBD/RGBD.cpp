/** @file RGBD.cpp
 *
 * \brief The methods, which might be useful when dealing with RGBD data
 * \author Michal Nowicki
 *
 */
#include "../include/RGBD/RGBD.h"

int RGBD::roundSize(double x, int size) {
	if (x < 0)
		x = 0;
	else if (x > size - 1)
		x = size;
	return round(x);
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
		std::vector<cv::Point2f> undistortedFeatures2D, cv::Mat depthImage) {

	// Assume standard distortion
	float cameraMatrix[3][3] = { { 517.3, 0, 318.6 }, { 0, 516.5, 255.3 }, { 0,
			0, 1 } };

	// Call method with additional parameters
	return keypoints2Dto3D(undistortedFeatures2D, depthImage,
			cv::Mat(3, 3, CV_32FC1, &cameraMatrix));
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
		std::vector<cv::Point2f> undistortedFeatures2D, cv::Mat depthImage,
		cv::Mat cameraMatrix, int startingID) {

	// Lets create 3D points
	std::vector<Eigen::Vector3f> features3D(undistortedFeatures2D.size() - startingID);
	int i = 0;
	for (std::vector<cv::Point2f>::iterator it = undistortedFeatures2D.begin() + startingID;
			it != undistortedFeatures2D.end(); ++it) {

		features3D[i] = point2Dto3D(*it, depthImage, cameraMatrix);
		i++;
	}

	return features3D;
}

Eigen::Vector3f RGBD::point2Dto3D(cv::Point2f feature2D,
		cv::Mat depthImage, cv::Mat cameraMatrix) {

	// Feature are extracted with subpixel precision, so find closest pixel
	int uRounded = roundSize(feature2D.x, depthImage.cols);
	int vRounded = roundSize(feature2D.y, depthImage.rows);

	// Convert it using the scaling of depth image
	float Z = depthImage.at<uint16_t>(vRounded, uRounded) / RGBD::depthScale;

	// Compute the feature position in normalized image coordinates
	float u = (feature2D.x - cameraMatrix.at<float>(0, 2))
			/ cameraMatrix.at<float>(0, 0);
	float v = (feature2D.y - cameraMatrix.at<float>(1, 2))
			/ cameraMatrix.at<float>(1, 1);

	// Create 3D feature
	return Eigen::Vector3f(u * Z, v * Z, Z);
}

Eigen::Vector3f RGBD::point2Dto3D(cv::Point2f feature2D,
		float depth, cv::Mat cameraMatrix) {

	// Feature are extracted with subpixel precision, so find closest pixel
	int uRounded = round(feature2D.x);
	int vRounded = round(feature2D.y);

	// Compute the feature position in normalized image coordinates
	float u = (feature2D.x - cameraMatrix.at<float>(0, 2))
			/ cameraMatrix.at<float>(0, 0);
	float v = (feature2D.y - cameraMatrix.at<float>(1, 2))
			/ cameraMatrix.at<float>(1, 1);

	// Create 3D feature
	return Eigen::Vector3f(u * depth, v * depth, depth);
}

// Project 3D points onto images
std::vector<cv::Point2f> RGBD::points3Dto2D(std::vector<Eigen::Vector3f> features3D, cv::Mat cameraMatrix) {

	std::vector<cv::Point2f> features2D(features3D.size());
	int i=0;
	for (std::vector<Eigen::Vector3f>::iterator it = features3D.begin(); it!=features3D.end(); ++it, i++)
	{
		features2D[i] = RGBD::point3Dto2D(*it, cameraMatrix);
	}
	return features2D;
}

cv::Point2f RGBD::point3Dto2D(Eigen::Vector3f feature3D, cv::Mat cameraMatrix) {
	float u = feature3D.x() * cameraMatrix.at<float>(0, 0) / feature3D.z()
			+ cameraMatrix.at<float>(0, 2);
	float v = feature3D.y() * cameraMatrix.at<float>(1, 1) / feature3D.z()
			+ cameraMatrix.at<float>(1, 2);
	return cv::Point2f(u, v);
}

void RGBD::removeFeaturesWithoutDepth(std::vector<cv::KeyPoint> &features,
		cv::Mat depthImage) {
	// Lambda expression
	auto it =
			std::remove_if(features.begin(), features.end(),
					[depthImage](cv::KeyPoint kp) {
						if (depthImage.at<uint16_t>(kp.pt) / RGBD::depthScale > 0.0) {
							return false;
						}
						return true;
					});
	features.erase(it, features.end());
}

void RGBD::removeMapFeaturesWithoutDepth(std::vector<MapFeature> &features,
		cv::Mat depthImage, float additionalDistance, std::vector<int> &frameIds, std::vector<float_type> &angles) {

	std::vector<MapFeature>::iterator featuresIter = features.begin();
	std::vector<int>::iterator frameIdsIter = frameIds.begin();
	std::vector<float_type>::iterator anglesIter = angles.begin();

	for (;featuresIter!=features.end();)
	{
		int uRounded = roundSize(featuresIter->u, depthImage.cols);
		int vRounded = roundSize(featuresIter->v, depthImage.rows);

		if (depthImage.at<uint16_t>(cv::Point2f(uRounded, vRounded))
				/ RGBD::depthScale
				<= featuresIter->position.z() - additionalDistance) {
			featuresIter = features.erase(featuresIter);
			frameIdsIter = frameIds.erase(frameIdsIter);
			anglesIter = angles.erase(anglesIter);
		}
		else {
			++featuresIter;
			++frameIdsIter;
			++anglesIter;
		}
	}
}

std::vector<cv::Point2f> RGBD::removeImageDistortion(
		std::vector<cv::KeyPoint>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs) {

	// Check if the vector is not empty
	if (features.size() == 0)
		return std::vector<cv::Point2f>();

	// Convert to points2D and then to Mat
	std::vector<cv::Point2f> points2D;
	cv::KeyPoint::convert(features, points2D);
	cv::Mat pointsDisorted(points2D), pointsUndistorted;

	// Undistortion
	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
			distCoeffs);

	std::vector<cv::Point2f> returnVector;
	for (int i = 0; i < pointsUndistorted.rows; i++) {

		// u = (u_normalized*fx) + cx
		float u = pointsUndistorted.at<cv::Vec2f>(i)[0]
				* cameraMatrix.at<float>(0, 0) + cameraMatrix.at<float>(0, 2);

		// v = (v_normalized*fy) + cy
		float v = pointsUndistorted.at<cv::Vec2f>(i)[1]
				* cameraMatrix.at<float>(1, 1) + cameraMatrix.at<float>(1, 2);
		returnVector.push_back(cv::Point2f(u, v));
	}
	return returnVector;
}

std::vector<Eigen::Vector3f> RGBD::imageToPointCloud(cv::Mat rgbImage,
		cv::Mat depthImage, cv::Mat cameraMatrix, Eigen::Matrix4f pose) {
	std::vector<Eigen::Vector3f> pointCloud;

	for(int j = 0;j < rgbImage.rows;j++){
	    for(int i = 0;i < rgbImage.cols;i++){

	    	float depth = depthImage.at<uint16_t>(cv::Point2f(i,j)) / RGBD::depthScale;

	    	Eigen::Vector3f point3D = point2Dto3D(cv::Point2f(i,j),
	    			depth, cameraMatrix);

	    	point3D = pose.block<3,3>(0,0) * point3D + pose.block<3,1>(0,3);

	    	if ( point3D.z() > 0.0001) {
	    		pointCloud.push_back(point3D);
	    	}
	    }
	}
	return pointCloud;
}

void RGBD::saveToFile(std::vector<Eigen::Vector3f> pointCloud, std::string fileName, bool first, Eigen::Matrix4f tmpPose)
{
	std::ofstream fileToSave;

	if (first) {
		fileToSave.open(fileName);
		fileToSave << "NODE 0.0 0.0 0.0 0.0 0.0 0.0" << std::endl;
	}
	else {
		fileToSave.open(fileName,  std::ofstream::out | std::ofstream::app);
	}
	std::cout << "Writing NODE" << std::endl;

	// OCTOMAP:
	// he keyword NODE is followed by the 6D pose of the laser origin of the 3D scan
	//(coordinates are regarded as SI units: meter for translation & rad for angles. x points forward, y left, z up.
	//roll, pitch, and yaw angles are around the axes x, y, z respectively).
	Eigen::Vector3f eulerAnglesRPY = tmpPose.block<3,3>(0,0).eulerAngles(0, 1, 2);

//	fileToSave << "NODE " << tmpPose(0, 3) << " " << tmpPose(1, 3) << " "
//			<< tmpPose(2, 3) << " " << eulerAnglesRPY(0) << " "
//			<< eulerAnglesRPY(1) << " " << eulerAnglesRPY(2) << std::endl;

	for (int i=0;i<pointCloud.size();i++) {
		fileToSave << pointCloud[i].x() << " " << pointCloud[i].y() << " " << pointCloud[i].z() << std::endl;
	}
	fileToSave.close();
}
