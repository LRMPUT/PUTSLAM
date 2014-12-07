#include "../include/TransformEst/kabschEst.h"
#include <memory>
#include <stdexcept>

using namespace putslam;

/// A single instance of Kabsch Estimator
KabschEst::Ptr kabsch;

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}

KabschEst::KabschEst(void) : name("Kabsch Estimator") {

}

const std::string& KabschEst::getName() const {
    return name;
}

/// compute transformation using two set of keypoints
Mat34& KabschEst::computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB){

    Eigen::Vector3d centerOfMassA, centerOfMassB;
    transformation.setIdentity();
    if (setA.rows()==0) return transformation;

    // Calculating center of mass
    for (int i=0;i<3;i++) {
        centerOfMassA[i] = setA.col(i).mean();
        centerOfMassB[i] = setB.col(i).mean();
    }

    // Moving to center of mass
    Eigen::MatrixXd setAnew(setA.rows(), setA.cols()); Eigen::MatrixXd setBnew(setB.rows(), setB.cols());
    for(int j=0;j<setA.rows();j++) {
            setAnew.row(j) = setA.row(j) - centerOfMassA.transpose();
            setBnew.row(j) = setB.row(j) - centerOfMassB.transpose();
    }

    // A = setAnew * setBnew
    Eigen::MatrixXd A = setAnew.transpose() * setBnew;

    // SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixU();
    Eigen::MatrixXd W = svd.matrixV();

    // Ensuring it is right-handed coordinate system
    Eigen::Matrix3d U = Eigen::MatrixXd::Identity(3,3);
    U(2,2) = sgn((A.determinant()!=0) ? A.determinant() : 1);

    // Optimal rotation matrix
    U = W * U * V.transpose();

    // Computing the translation
    Eigen::Vector3d T;
    T = -centerOfMassA;
    T = U*T;
    T += centerOfMassB;

    // Optimal transformation
    transformation.matrix().block<3,3>(0,0) = U.block<3,3>(0,0);
    transformation.matrix().block<3,1>(0,3) = T.head<3>();
    return transformation;
}

putslam::TransformEst* putslam::createKabschEstimator(void) {
    kabsch.reset(new KabschEst());
    return kabsch.get();
}
