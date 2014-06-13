#include "../include/Grabber/xtion_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
XtionGrabber::Ptr grabber;

XtionGrabber::XtionGrabber(void) : Grabber("Xtion Grabber", TYPE_PRIMESENSE) {

}

const std::string& XtionGrabber::getName() const {
    return name;
}

const PointCloud& XtionGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& XtionGrabber::getSensorFrame(void) const {
    return sensor_frame;
}

void XtionGrabber::grab(void) {
    Point3D point;
    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
    point.x = 1.2; point.y = 3.4; point.z = 5.6;
    cloud.push_back(point);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/// run grabber thread
void XtionGrabber::calibrate(void) {

}

putslam::Grabber* putslam::createGrabberXtion(void) {
    grabber.reset(new XtionGrabber());
    return grabber.get();
}

putslam::Grabber* putslam::createGrabberXtion(std::string configFile) {
    grabber.reset(new XtionGrabber(configFile));
    return grabber.get();
}
