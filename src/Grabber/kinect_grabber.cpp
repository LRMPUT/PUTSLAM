#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>

using namespace handest;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabber;

KinectGrabber::KinectGrabber(void) : name("Kinect Grabber") {

}

const std::string& KinectGrabber::getName() const {
	return name;
}

void KinectGrabber::getCloud(Point3D::Cloud& current_cloud) const {
    current_cloud = cloud;
}

void KinectGrabber::grab(void) {
    Point3D point;
    point.colour.r = 255; point.colour.g = 0; point.colour.b = 0; point.colour.a = 255;
    point.position.v[0] = 1.2; point.position.v[1] = 3.4; point.position.v[2] = 5.6;
	cloud.push_back(point);
}

handest::Grabber* handest::createGrabberKinect(void) {
	grabber.reset(new KinectGrabber());
	return grabber.get();
}
