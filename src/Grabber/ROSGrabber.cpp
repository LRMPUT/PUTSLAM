#ifdef BUILD_WITH_ROS

// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "../../include/Grabber/ROSGrabber.h"

#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "ros/topic_manager.h"

/// A single instance of Kinect grabber
ROSGrabber::Ptr grabberROS;
//"/camera/rgb/image_color"   "/camera/depth/image"
ROSGrabber::ROSGrabber(void) : Grabber("ROS Grabber", TYPE_PRIMESENSE, MODE_BUFFER), imageRGB_sub(nh, "/rgb/image_raw", 10000), imageDepth_sub(nh, "/depth/image_raw", 10000), sync(MySyncPolicy(10000), imageRGB_sub, imageDepth_sub) {
	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/ROSModel.xml");
	if (config.ErrorID())
		std::cout << "Unable to load ROS Grabber config file: ROSModel.xml \n";
	config.FirstChildElement("parameters")->QueryIntAttribute("imageDepthScale", &imageDepthScale);
	config.FirstChildElement("parameters")->QueryIntAttribute("maxProcessFrames", &maxProcessFrames);
	sync.registerCallback(boost::bind(&ROSGrabber::callback, this, _1, _2));
	iterate = 1;
	lastReadId = -1;
}

ROSGrabber::ROSGrabber(ros::NodeHandle nh) : Grabber("ROS Grabber", TYPE_PRIMESENSE, MODE_BUFFER), imageRGB_sub(nh, "/rgb/image_raw", 10000), imageDepth_sub(nh, "/depth/image_raw", 10000), sync(MySyncPolicy(10000), imageRGB_sub, imageDepth_sub) {
	this->nh = nh;
	tinyxml2::XMLDocument config;
	config.LoadFile("../../resources/ROSModel.xml");
	if (config.ErrorID())
		std::cout << "Unable to load ROS Grabber config file: ROSModel.xml \n";
	config.FirstChildElement("parameters")->QueryIntAttribute("imageDepthScale", &imageDepthScale);
	config.FirstChildElement("parameters")->QueryIntAttribute("maxProcessFrames", &maxProcessFrames);
	sync.registerCallback(boost::bind(&ROSGrabber::callback, this, _1, _2));
	iterate = 1;
	lastReadId = -1;
}

void ROSGrabber::callback(const sensor_msgs::ImageConstPtr& imageRGB, const sensor_msgs::ImageConstPtr& imageDepth)
{
	cv_bridge::CvImagePtr cv_RGB_ptr, cv_Depth_ptr;
	try
	{
		cv_RGB_ptr = cv_bridge::toCvCopy(imageRGB, sensor_msgs::image_encodings::BGR8);
		if(imageDepth->encoding == "16UC1")
		cv_Depth_ptr = cv_bridge::toCvCopy(imageDepth, sensor_msgs::image_encodings::TYPE_16UC1);
		else if(imageDepth->encoding == "32FC1")
		cv_Depth_ptr = cv_bridge::toCvCopy(imageDepth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	double ns = imageRGB->header.stamp.nsec;
	while(ns>1) {
		ns = ns/10;
	}
	try
	{
		mtx.lock();
		this->sensorFrame.readId = iterate;
		iterate++;
		this->sensorFrame.timestamp = ((double)imageRGB->header.stamp.sec) + ns;
		this->sensorFrame.depthImageScale = imageDepthScale;
		this->sensorFrame.rgbImage = cv_RGB_ptr->image;
		this->sensorFrame.depthImage = cv_Depth_ptr->image;

		if (mode==MODE_BUFFER) {
			sensorFrames.push(sensorFrame);
		}
		mtx.unlock();
	}
	catch (cv::Exception& e)
	{
		ROS_ERROR("cv exception: %s", e.what());
		return;
	}
}

const std::string& ROSGrabber::getName() const {
	return name;
}

const PointCloud& ROSGrabber::getCloud(void) const {
	return cloud;
}

bool ROSGrabber::grab(void) {
	if(lastReadId == -1 || lastReadId < maxProcessFrames)
		return true;
 	return false;
}

const SensorFrame& ROSGrabber::getSensorFrame(void) {
	bool waitForImage = true;
	ros::Rate r(60);	//loop is set to run at 30Hz
	while (waitForImage) {
		ros::spinOnce();
		mtx.lock();
		if (sensorFrames.size())
			waitForImage = false;
		mtx.unlock();
		r.sleep();
	}
	mtx.lock();
	//SensorFrame returnSensorFrame = sensorFrames.front();
	sensorFrame = sensorFrames.front();

	sensorFrames.pop();
	mtx.unlock();
	lastReadId = sensorFrame.readId;
	return sensorFrame;
}

void ROSGrabber::calibrate(void) {

}

ROSGrabber::~ROSGrabber(void) {

}

int ROSGrabber::grabberClose() {
	return 0;
}

Eigen::Matrix4f ROSGrabber::getStartingSensorPose()
{
	Eigen::Matrix4f initialSensorPose = Eigen::Matrix4f::Identity();
	return initialSensorPose;
}

putslam::Grabber* putslam::createGrabberROS(void) {
	grabberROS.reset(new ROSGrabber());
	return grabberROS.get();
}

putslam::Grabber* putslam::createGrabberROS(ros::NodeHandle nh) {
	grabberROS.reset(new ROSGrabber(nh));
	return grabberROS.get();
}
#endif
