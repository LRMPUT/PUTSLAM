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


// Out types
#include "../include/Map/featuresMap.h"

namespace RGBD {

/// Depth image scale
const double depthScale = 5000;

// Convert keypoints to 3D points using depth image
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::Point2f> undistortedFeatures2D,
		cv::Mat depthImage);
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::Point2f> undistortedFeatures2D,
		cv::Mat depthImage, cv::Mat cameraMatrix, int startingID = 0);

Eigen::Vector3f point2Dto3D(cv::Point2f undistortedFeatures2D,
		cv::Mat depthImage, cv::Mat cameraMatrix);
Eigen::Vector3f point2Dto3D(cv::Point2f undistortedFeatures2D,
		float depth, cv::Mat cameraMatrix);

// Project 3D points onto images
std::vector<cv::Point2f> points3Dto2D(std::vector<Eigen::Vector3f> features3D, cv::Mat cameraMatrix);

cv::Point2f point3Dto2D(Eigen::Vector3f feature3D, cv::Mat cameraMatrix);

// Rounds the (u,v) location to integer
int roundSize(double x, int size);

// Remove features without depth
void removeFeaturesWithoutDepth(std::vector<cv::KeyPoint> &features,
		cv::Mat depthImage);

void removeMapFeaturesWithoutDepth(std::vector<MapFeature> &features,
		cv::Mat depthImage, float additionalDistance, std::vector<int> &frameIds, std::vector<float_type> &angles);

std::vector<cv::Point2f> removeImageDistortion(
		std::vector<cv::KeyPoint>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs);

// Building a point cloud
std::vector<Eigen::Vector3f> imageToPointCloud(cv::Mat rgbImage, cv::Mat depthImage, cv::Mat cameraMatrix, Eigen::Matrix4f pose);

void saveToFile(std::vector<Eigen::Vector3f> pointCloud, std::string fileName,
		bool first = false, Eigen::Matrix4f tmpPose = Eigen::Matrix4f::Identity());

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
