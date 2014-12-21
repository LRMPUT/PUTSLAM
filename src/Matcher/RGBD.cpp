/** @file RGBD.cpp
 *
 * \brief The methods, which might be useful when dealing with RGBD data
 * \author Michal Nowicki
 *
 */
#include "../include/Matcher/RGBD.h"

// Convert Keypoints to 3D points
std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		std::vector<float> depth) {

	// Assume standard distortion
	float distortionCoeffs[5] = { -0.003632241033, -0.2716829622,
			0.001093892549, -0.001320103758, 0.8473502424 };
	float cameraMatrix[3][3] = { { 574.1240803, 0, 319.796104 }, { 0,
			573.9756877, 243.9607685 }, { 0, 0, 1 } };

	// Call method with additional parameters
	return keypoints2Dto3D(features, depth,
			cv::Mat(3, 3, CV_32FC1, &cameraMatrix),
			cv::Mat(1, 5, CV_32FC1, &distortionCoeffs));
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		std::vector<float> depth, cv::Mat cameraMatrix, cv::Mat distCoeffs) {

	// Convert to points2D and then to Mat
	std::vector<cv::Point2f> points2D;
	cv::KeyPoint::convert(features, points2D);
	cv::Mat pointsDisorted(points2D), pointsUndistorted;

	// Undistortion
	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
			distCoeffs);

	// Lets create 3D points
	std::vector<Eigen::Vector3f> features3D(pointsUndistorted.rows);
	for (int i = 0; i < pointsUndistorted.rows; i++) {
		float X = pointsUndistorted.at<cv::Vec2f>(i)[0] * depth[i];
		float Y = pointsUndistorted.at<cv::Vec2f>(i)[1] * depth[i];
		features3D[i] = Eigen::Vector3f(X, Y, depth[i]);
	}

	return features3D;
}

int RGBD::roundSize(double x, int size) {
	if (x < 0)
		x = 0;
	else if (x > size - 1)
		x = size;
	return round(x);
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		cv::Mat depthImage) {

	// Assume standard distortion
	float distortionCoeffs[5] = { -0.003632241033, -0.2716829622,
			0.001093892549, -0.001320103758, 0.8473502424 };
	float cameraMatrix[3][3] = { { 574.1240803, 0, 319.796104 }, { 0,
			573.9756877, 243.9607685 }, { 0, 0, 1 } };

	// Call method with additional parameters
	return keypoints2Dto3D(features, depthImage,
			cv::Mat(3, 3, CV_32FC1, &cameraMatrix),
			cv::Mat(1, 5, CV_32FC1, &distortionCoeffs));
}

std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		cv::Mat depthImage, cv::Mat cameraMatrix, cv::Mat distCoeffs) {

	// Convert to points2D and then to Mat
	std::vector<cv::Point2f> points2D;
	cv::KeyPoint::convert(features, points2D);
	cv::Mat pointsDisorted(points2D), pointsUndistorted;

	// Undistortion
	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
			distCoeffs);

	// Lets create 3D points
	std::vector<Eigen::Vector3f> features3D(pointsUndistorted.rows);
	for (int i = 0; i < pointsUndistorted.rows; i++) {
		float u = pointsUndistorted.at<cv::Vec2f>(i)[0];
		float v = pointsUndistorted.at<cv::Vec2f>(i)[1];
		float uRounded = roundSize(u, depthImage.cols);
		float vRounded = roundSize(v, depthImage.rows);
		float Z = depthImage.at<float>(vRounded, uRounded) / RGBD::depthScale;
		features3D[i] = Eigen::Vector3f(u * Z, v * Z, Z);
	}

	return features3D;
}


void RGBD::removeFeaturesWithoutDepth(std::vector<cv::KeyPoint> &features, cv::Mat depthImage)
{
	// Lambda expression
	auto it = std::remove_if (features.begin(), features.end(), [depthImage](cv::KeyPoint kp){
	    if (depthImage.at<uint16_t>(kp.pt) / RGBD::depthScale > 0.0) {
	        return false;
	    }
	    return true;
	});
	features.erase(it, features.end());
}

