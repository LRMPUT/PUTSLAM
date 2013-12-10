#include "../include/Tracker/trackerKLT.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Kanade-Lucas-Tomasi tracker
TrackerKLT::Ptr tracker;

TrackerKLT::TrackerKLT(void) : name("KLT_Tracker") {

}

const std::string& TrackerKLT::getName() const {
    return name;
}

/// Returns the current set of features
const ImageFeature::Seq& TrackerKLT::getFeatures(void) const {
    return this->features;
}

/// Reset tracking and find new set of features
void TrackerKLT::reset(){

}

/// Run tracking thread
void TrackerKLT::run(void) {

}

putslam::Tracker* putslam::createTrackerKLT(void) {
    tracker.reset(new TrackerKLT());
    return tracker.get();
}
