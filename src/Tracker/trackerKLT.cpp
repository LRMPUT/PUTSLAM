#include "Tracker/trackerKLT.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Kanade-Lucas-Tomasi tracker
TrackerKLT::Ptr tracker;

TrackerKLT::TrackerKLT(void) : Tracker("Lucas-Kanade-Tomasi Tracker"){

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
    frame_id = 0;
}

/// Run single tracking iteration
bool TrackerKLT::track(const SensorFrame& next_frame) {
    std::cout << "tracking frame: " << frame_id <<next_frame.timestamp << std::endl;
    frame_id++;
    if (!((frame_id+1)%10)){//check if we should continue tracking
        reset();
        return false;
    }
    else
        return true;
}

/// Compute homogenous transformation
const Mat34& TrackerKLT::computeTransform(void) {
    std::cout << "KLT: compute transformation\n";
    return transformation;
}

/// get Vertex: set of Keypoints/ point Cloud and sensor/robot pose
const VertexSE3& TrackerKLT::getVertex(void){
    return keypoint;
}

putslam::Tracker* putslam::createTrackerKLT(void) {
    tracker.reset(new TrackerKLT());
    return tracker.get();
}
