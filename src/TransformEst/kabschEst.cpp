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

/// Returns current transformation
const Mat34& KabschEst::getTransformation(void) const{
    return transformation;
}

/// compute transformation using two set of keypoints
void KabschEst::computeTransformation(ImageFeature::Seq& keypointA, ImageFeature::Seq& keypointB){

}

putslam::TransformEst* putslam::createKapschEstimator(void) {
    kabsch.reset(new KabschEst());
    return kabsch.get();
}
