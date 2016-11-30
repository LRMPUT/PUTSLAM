#include "TransformEst/ICPEst.h"
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

/// compute transformation using two set of keypoints
Mat34& ICPEst::computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB){
    std::cout << setA.rows() << setB.rows() << "\n";
    return transformation;
}

putslam::TransformEst* putslam::createICPEstimator(void) {
    icp_est.reset(new ICPEst());
    return icp_est.get();
}
