#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabberK;

KinectGrabber::KinectGrabber(void) : Grabber("Kinect Grabber", TYPE_PRIMESENSE) {

}

const std::string& KinectGrabber::getName() const {
	return name;
}

const PointCloud& KinectGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& KinectGrabber::getSensorFrame(void) {
    return sensorFrame;
}

bool KinectGrabber::grab(void) {
    Point3D point;
    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
    point.x = 1.2; point.y = 3.4; point.z = 5.6;
	cloud.push_back(point);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

/// run grabber thread
void KinectGrabber::calibrate(void) {

}

int KinectGrabber::grabberClose(){
    return 0;
}

putslam::Grabber* putslam::createGrabberKinect(void) {
    grabberK.reset(new KinectGrabber());
    return grabberK.get();
}

putslam::Grabber* putslam::createGrabberKinect(std::string configFile) {
    grabberK.reset(new KinectGrabber(configFile));
    return grabberK.get();
}

