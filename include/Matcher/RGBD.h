/** @file RGBD.d
 *
 * \brief The methods, which might be useful when dealing with RGBD data
 * \author Michal Nowicki
 *
 */
#ifndef _RGBD_H
#define _RGBD_H

// Ogolne + STL
#include <iostream>
#include <vector>
#include <string>

// Podst PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

// OpenCV
#include "opencv/cv.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace RGBD {

// Convert keypoints to 3D points using vector of float as depth
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		std::vector<float> depth);
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		std::vector<float> depth, cv::Mat cameraMatrix, cv::Mat distCoeffs);

// Convert keypoints to 3D points using depth image
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		cv::Mat depthImage);
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::KeyPoint> features,
		cv::Mat depthImage, cv::Mat cameraMatrix, cv::Mat distCoeffs);


// Rounds the (u,v) location to integer
int roundSize(double x, int size);

// Remove features without depth
void removeFeaturesWithoutDepth(std::vector<cv::KeyPoint> &features, cv::Mat depthImage);


// Building a point cloud
//static Eigen::Vector3f point2Dto3D(cv::Point2f p, float z, cv::Mat cameraMatrix, cv::Mat distCoeffs);

//	static Eigen::Vector3f simplePoint2Dto3D(cv::Point2f p, float z, CalibrationParameters cameraParams);
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BuildPointCloudFromRGBD(
//			cv::Mat rgbImage, cv::Mat dImage, double depthInvScale, CalibrationParameters cameraParameters);
//
//	// Operations on pointCloud
//	void TransformSelf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
//			Eigen::Matrix4f &transformation);
//	void TransformSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//			Eigen::Matrix4f &transformation);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr Transform(
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//			Eigen::Matrix4f &transformation);
}

#endif