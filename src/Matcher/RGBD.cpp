/** @file RGBD.cpp
 *
 * \brief The methods, which might be useful when dealing with RGBD data
 * \author Michal Nowicki
 *
 */
#include "../include/Matcher/RGBD.h"

//// Convert Keypoints to 3D points
//std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
//		std::vector<cv::KeyPoint> features, std::vector<float> depth) {
//
//	// Assume standard distortion
//	float distortionCoeffs[5] = { -0.0410, 0.3286, 0.0087, 0.0051, -0.5643 };
//	float cameraMatrix[3][3] = { { 517.3, 0, 318.6 }, { 0, 516.5, 255.3 }, { 0,
//			0, 1 } };
//
//	// Call method with additional parameters
//	return keypoints2Dto3D(features, depth,
//			cv::Mat(3, 3, CV_32FC1, &cameraMatrix),
//			cv::Mat(1, 5, CV_32FC1, &distortionCoeffs));
//}
//
//std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
//		std::vector<cv::KeyPoint> features, std::vector<float> depth,
//		cv::Mat cameraMatrix, cv::Mat distCoeffs) {
//
//	// Convert to points2D and then to Mat
//	std::vector<cv::Point2f> points2D;
//	cv::KeyPoint::convert(features, points2D);
//	cv::Mat pointsDisorted(points2D), pointsUndistorted;
//
//	// Undistortion
//	cv::undistortPoints(pointsDisorted, pointsUndistorted, cameraMatrix,
//			distCoeffs);
//
//	// Lets create 3D points
//	std::vector<Eigen::Vector3f> features3D(pointsUndistorted.rows);
//	for (int i = 0; i < pointsUndistorted.rows; i++) {
//		float X = pointsUndistorted.at<cv::Vec2f>(i)[0] * depth[i];
//		float Y = pointsUndistorted.at<cv::Vec2f>(i)[1] * depth[i];
//		features3D[i] = Eigen::Vector3f(X, Y, depth[i]);
//	}
//
//	return features3D;
//}

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
	float distortionCoeffs[5] = { -0.0410, 0.3286, 0.0087, 0.0051, -0.5643 };
	float cameraMatrix[3][3] = { { 517.3, 0, 318.6 }, { 0, 516.5, 255.3 }, { 0,
			0, 1 } };

	// Call method with additional parameters
	return keypoints2Dto3D(undistortedFeatures2D, depthImage,
			cv::Mat(3, 3, CV_32FC1, &cameraMatrix),
			cv::Mat(1, 5, CV_32FC1, &distortionCoeffs));
}


std::vector<Eigen::Vector3f> RGBD::keypoints2Dto3D(
		std::vector<cv::Point2f> undistortedFeatures2D, cv::Mat depthImage,
		cv::Mat cameraMatrix, cv::Mat distCoeffs) {

	// Lets create 3D points
	std::vector<Eigen::Vector3f> features3D(undistortedFeatures2D.size());
	int i = 0;
	for (std::vector<cv::Point2f>::iterator it = undistortedFeatures2D.begin(); it!=undistortedFeatures2D.end(); ++it)
	{
		// Feature are extracted with subpixel precision, so find closest pixel
		int uRounded = roundSize(it->x, depthImage.cols);
		int vRounded = roundSize(it->y, depthImage.rows);

		// Convert it using the scaling of depth image
		float Z = depthImage.at<uint16_t>(vRounded, uRounded)
				/ RGBD::depthScale;

		// Compute the feature position in normalized image coordinates
		float u = (it->x - cameraMatrix.at<float>(0, 2)) / cameraMatrix.at<float>(0, 0);
		float v = (it->y - cameraMatrix.at<float>(1, 2)) / cameraMatrix.at<float>(1, 1);

		// Create 3D feature
		features3D[i] = Eigen::Vector3f(u * Z, v * Z, Z);
		i++;
	}

	return features3D;
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

std::vector<cv::Point2f> RGBD::removeImageDistortion(
		std::vector<cv::KeyPoint>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs) {

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
