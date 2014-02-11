#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabber;

KinectGrabber::KinectGrabber(void) : Grabber("Kinect Grabber", TYPE_PRIMESENSE) {

}

const std::string& KinectGrabber::getName() const {
	return name;
}

const PointCloud& KinectGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& KinectGrabber::getSensorFrame(void) const {
    return sensor_frame;
}

void KinectGrabber::grab(void) {
    Point3D point;
    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
    point.x = 1.2; point.y = 3.4; point.z = 5.6;
	cloud.push_back(point);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/// run grabber thread
void KinectGrabber::calibrate(void) {

}

putslam::Grabber* putslam::createGrabberKinect(void) {
	grabber.reset(new KinectGrabber());
	return grabber.get();
}
