#include "Grabber/kinectGrabber.h"

#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;
/// A single instance of Kinect grabber
KinectGrabber::Ptr grabberK;

KinectGrabber::KinectGrabber(void) : Grabber("Kinect Grabber", TYPE_PRIMESENSE, MODE_BUFFER)
  #ifdef BUILD_KINECT
, device(freenect.createDevice<MyFreenectDevice>(0))
    #endif
    {
    #ifdef BUILD_KINECT
    usleep(1000000);
    device.startVideo();
    device.startDepth();
    cv::Mat depthMat(cv::Size(640,480),CV_16UC1);
    device.getDepth(depthMat);
    sensorFrame.depthImage = depthMat.clone();
    frameNo=0;
    #endif
    sensorFrame.depthImageScale = model.config.depthImageScale;
}

const std::string& KinectGrabber::getName() const {
	return name;
}

const PointCloud& KinectGrabber::getCloud(void) const {
    return cloud;
}

bool KinectGrabber::grab(void) {
    usleep(5000);
    #ifdef BUILD_KINECT
    device.getVideo(this->sensorFrame.rgbImage);
    cv::Mat depthMat(cv::Size(640,480),CV_16UC1);

    device.getDepth(depthMat);
    sensorFrame.depthImage = depthMat.clone();
    #endif
    return true;

}

const SensorFrame& KinectGrabber::getSensorFrame(void) {
    return sensorFrame;
}

/// run grabber thread
void KinectGrabber::calibrate(void) {

}

int KinectGrabber::grabberClose(){
    return 0;
}

Eigen::Matrix4f KinectGrabber::getStartingSensorPose()
{
	return Eigen::Matrix4f::Identity();
}


putslam::Grabber* putslam::createGrabberKinect(void) {
    grabberK.reset(new KinectGrabber());
    return grabberK.get();
}

putslam::Grabber* putslam::createGrabberKinect(std::string configFile, Grabber::Mode mode) {
    grabberK.reset(new KinectGrabber(configFile, mode));
    return grabberK.get();
}

