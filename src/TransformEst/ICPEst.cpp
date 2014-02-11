#include "../include/TransformEst/ICPEst.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Generalized Estimator
ICPEst::Ptr icp_est;

ICPEst::ICPEst(void) : name("Kabsch Estimator") {

}

const std::string& ICPEst::getName() const {
    return name;
}

/// Returns current transformation
void ICPEst::setInputClouds(PointCloud cloudA, PointCloud cloudB){

}

/// compute transformation using two set of keypoints
const Mat34& ICPEst::computeTransformation(void){
    return transformation;
}

putslam::TransformEst* putslam::createICPEstimator(void) {
    icp_est.reset(new ICPEst());
    return icp_est.get();
}
