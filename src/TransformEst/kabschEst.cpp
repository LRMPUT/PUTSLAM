#include "../include/TransformEst/kabschEst.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Kabsch Estimator
KabschEst::Ptr kabsch;

KabschEst::KabschEst(void) : name("Kabsch Estimator") {

}

const std::string& KabschEst::getName() const {
    return name;
}

void KabschEst::setInputKeypoints(ImageFeature::Seq& keypointA, ImageFeature::Seq& keypointB){

}

/// compute transformation using two set of keypoints
const Mat34& KabschEst::computeTransformation(void){
    return transformation;
}

putslam::TransformEst* putslam::createKapschEstimator(void) {
    kabsch.reset(new KabschEst());
    return kabsch.get();
}
