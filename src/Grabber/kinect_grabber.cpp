#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabber;

KinectGrabber::KinectGrabber(void) : name("Kinect Grabber"), type(TYPE_PRIMESENSE) {

}

const std::string& KinectGrabber::getName() const {
	return name;
}

const Point3D::Cloud& KinectGrabber::getCloud(void) const {
    return cloud;
}

const Image& KinectGrabber::getImage(void) const {
    return image;
}

void KinectGrabber::grab(void) {
    Point3D point;
    point.colour.r = 255; point.colour.g = 0; point.colour.b = 0; point.colour.a = 255;
    point.position.v[0] = 1.2; point.position.v[1] = 3.4; point.position.v[2] = 5.6;
	cloud.push_back(point);
}

/// run grabber thread
void KinectGrabber::run(void) {

}

putslam::Grabber* putslam::createGrabberKinect(void) {
	grabber.reset(new KinectGrabber());
	return grabber.get();
}
