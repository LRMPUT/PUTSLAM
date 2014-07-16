#include "../include/Grabber/ptgrey_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
PtgreyGrabber::Ptr grabberP;

PtgreyGrabber::PtgreyGrabber(void) : Grabber("Ptgrey Grabber", TYPE_PRIMESENSE) {

}

PtgreyGrabber::PtgreyGrabber(std::string modelFilename) : Grabber("Ptgrey Grabberr", TYPE_PRIMESENSE), model(modelFilename){

}

const std::string& PtgreyGrabber::getName() const {
    return name;
}

const PointCloud& PtgreyGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& PtgreyGrabber::getSensorFrame(void) const {
    return sensor_frame;
}

void PtgreyGrabber::grab(void) {
    Point3D point;
    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
    point.x = 1.2; point.y = 3.4; point.z = 5.6;
	cloud.push_back(point);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/// run grabber thread
void PtgreyGrabber::calibrate(void) {

}

int PtgreyGrabber::grabberClose(){
    return 0;
}

putslam::Grabber* putslam::createGrabberPtgrey(void) {
    grabberP.reset(new PtgreyGrabber());
    return grabberP.get();
}

putslam::Grabber* putslam::createGrabberPtgrey(std::string configFile) {
    grabberP.reset(new PtgreyGrabber(configFile));
    return grabberP.get();
}

