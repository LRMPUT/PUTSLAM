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
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include "pcl/io/pcd_io.h"

// OpenCV
#include "opencv/cv.h"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include "Defs/eigen3.h"

// Out types
#include "../include/Defs/putslam_defs.h"
//#include "../include/Map/featuresMap.h"

namespace RGBD {

/// Depth image scale
//const double depthScale = 5000;

// Convert keypoints to 3D points using depth image
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::Point2f> undistortedFeatures2D,
		cv::Mat depthImage, double depthImageScale);
std::vector<Eigen::Vector3f> keypoints2Dto3D(std::vector<cv::Point2f> undistortedFeatures2D,
		cv::Mat depthImage, cv::Mat cameraMatrix, double depthImageScale, int startingID = 0);

Eigen::Vector3f point2Dto3D(cv::Point2f undistortedFeatures2D,
		cv::Mat depthImage, cv::Mat cameraMatrix, double depthImageScale);
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

void removeMapFeaturesWithoutDepth(std::vector<putslam::MapFeature> &features,
        cv::Mat depthImage, float additionalDistance, std::vector<int> &frameIds, std::vector<putslam::float_type> &angles, double depthImageScale);

std::vector<cv::Point2f> removeImageDistortion(
		std::vector<cv::KeyPoint>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs);

std::vector<cv::Point2f> removeImageDistortion(
		std::vector<cv::Point2f>& features, cv::Mat cameraMatrix,
		cv::Mat distCoeffs);

// Building a point cloud
std::vector<Eigen::Vector3f> imageToPointCloud(cv::Mat rgbImage,
		cv::Mat depthImage, cv::Mat cameraMatrix, Eigen::Matrix4f pose,
		double depthImageScale);
std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3i>> imageToColorPointCloud(
		cv::Mat rgbImage, cv::Mat depthImage, cv::Mat cameraMatrix,
		Eigen::Matrix4f pose, double depthImageScale);

void saveToFile(std::vector<Eigen::Vector3f> pointCloud, std::string fileName,
bool first = false, Eigen::Matrix4f tmpPose = Eigen::Matrix4f::Identity());

///compute normal
putslam::Vec3 computeNormal(const cv::Mat& depthImage, int u, int v, const cv::Mat& cameraMatrix, double depthImageScale);

/// compute normals to rgbd features
template<class T>
void computeNormals(const cv::Mat& depthImage, T& features, const cv::Mat& cameraMatrix, double depthImageScale){
    for(auto it = features.begin();it!=features.end();it++){
        it->normal = computeNormal(depthImage,(int)it->u, (int)it->v, cameraMatrix, depthImageScale);
    }
}

//compute rgb gradient
putslam::Vec3 computeRGBGradient(const cv::Mat& rgbImage, const cv::Mat& depthImage, int u, int v, const cv::Mat& cameraMatrix, double depthImageScale);

/// compute rgbd gradients
template<class T>
void computeRGBGradients(const cv::Mat& rgbImage, const cv::Mat& depthImage, T& features, const cv::Mat& cameraMatrix, double depthImageScale){
    for(auto it = features.begin();it!=features.end();it++){
        it->RGBgradient = computeRGBGradient(rgbImage, depthImage, (int)it->u, (int)it->v, cameraMatrix, depthImageScale);
    }
}

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
