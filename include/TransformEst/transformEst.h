/** @file transformEst.h
 *
 * Feature-based transformation estimation interface
 *
 */

#ifndef _TRANSFORMEST_H_
#define _TRANSFORMEST_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Transformation Estimator interface
    class TransformEst {
        public:

            /// Name of the estimator
            virtual const std::string& getName() const = 0;

            /// compute transformation using two set of keypoints
            virtual Mat34& computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB) = 0;

            /// Virtual descrutor
            virtual ~TransformEst() {}

        /// Compute uncertainty matrix [6x6] (fi,psi,theta,x,y,z)
        virtual const Mat66& computeUncertainty(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation) {
            Mat66 dgdTheta; dgdTheta.setZero();
            Quaternion q(transformation.rotation());
            const double& q0 = q.w();
            const double& q1 = q.x();
            const double& q2 = q.y();
            const double& q3 = q.z();
            double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)); // r32/r33
            double pitch = asin(2*(q0*q2-q3*q1));
            double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
            float_type x = transformation.matrix()(0,3); float_type y = transformation.matrix()(1,3); float_type z = transformation.matrix()(2,3);

            Eigen::MatrixXd Cx(2*3*setA.rows(),2*3*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*3*setA.rows(), 2*3*setA.rows());
            Eigen::MatrixXd dgdX(2*3*setA.rows(),6);
            float_type k = 1.0/setA.rows();
            for (size_t i=0;i<setA.rows();i++){
                float_type xa = setA(i,0); float_type ya = setA(i,1); float_type za = setA(i,2);
                float_type xb = setB(i,0); float_type yb = setB(i,1); float_type zb = setB(i,2);

                dgdTheta(0,0) += 2.0;
                dgdTheta(0,1) += 0;
                dgdTheta(0,2) += 0;
                dgdTheta(0,3) += (2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb;
                dgdTheta(0,4) += -(2.0)*xb*cos(yaw)*sin(pitch)+(2.0)*cos(pitch)*cos(yaw)*zb*cos(roll)+(2.0)*cos(pitch)*sin(roll)*cos(yaw)*yb;
                dgdTheta(0,5) += -(2.0)*cos(pitch)*xb*sin(yaw)+(2.0)*zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb;

                dgdTheta(1,0) += 0.0;
                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += 0.0;
                dgdTheta(1,3) += -(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb-(2.0)*yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw));
                dgdTheta(1,4) += -(2.0)*xb*sin(pitch)*sin(yaw)+(2.0)*cos(pitch)*sin(roll)*sin(yaw)*yb+(2.0)*cos(pitch)*zb*cos(roll)*sin(yaw);
                dgdTheta(1,5) += (2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+(2.0)*cos(pitch)*xb*cos(yaw);

                dgdTheta(2,0) += 0.0;
                dgdTheta(2,1) += 0.0;
                dgdTheta(2,2) += 2.0;
                dgdTheta(2,3) += (2.0)*cos(pitch)*cos(roll)*yb-(2.0)*cos(pitch)*sin(roll)*zb;
                dgdTheta(2,4) += -(2.0)*zb*cos(roll)*sin(pitch)-(2.0)*sin(roll)*sin(pitch)*yb-(2.0)*cos(pitch)*xb;
                dgdTheta(2,5) += 0.0;

                dgdTheta(3,0) += (2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb;
                dgdTheta(3,1) += -(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb-(2.0)*yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw));
                dgdTheta(3,2) += (2.0)*cos(pitch)*cos(roll)*yb-(2.0)*cos(pitch)*sin(roll)*zb;
                dgdTheta(3,3) += -(2.0)*(zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))+(2.0)*pow((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb,2.0)-(2.0)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)+(2.0)*pow(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb,2.0)+(2.0)*pow((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)),2.0);
                dgdTheta(3,4) += -(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)+(2.0)*(sin(roll)*zb*sin(pitch)-cos(roll)*sin(pitch)*yb)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)-(2.0)*(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb)*(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb)+(2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*(cos(pitch)*cos(yaw)*cos(roll)*yb-cos(pitch)*sin(roll)*cos(yaw)*zb)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*(cos(pitch)*cos(roll)*sin(yaw)*yb-cos(pitch)*sin(roll)*zb*sin(yaw))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);
                dgdTheta(3,5) += (2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))-(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);

                dgdTheta(4,0) += -(2.0)*xb*cos(yaw)*sin(pitch)+(2.0)*cos(pitch)*cos(yaw)*zb*cos(roll)+(2.0)*cos(pitch)*sin(roll)*cos(yaw)*yb;
                dgdTheta(4,1) += -(2.0)*xb*sin(pitch)*sin(yaw)+(2.0)*cos(pitch)*sin(roll)*sin(yaw)*yb+(2.0)*cos(pitch)*zb*cos(roll)*sin(yaw);
                dgdTheta(4,2) += -(2.0)*zb*cos(roll)*sin(pitch)-(2.0)*sin(roll)*sin(pitch)*yb-(2.0)*cos(pitch)*xb;
                dgdTheta(4,3) += -(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)+(2.0)*(sin(roll)*zb*sin(pitch)-cos(roll)*sin(pitch)*yb)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)-(2.0)*(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb)*(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb)+(2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*(cos(pitch)*cos(yaw)*cos(roll)*yb-cos(pitch)*sin(roll)*cos(yaw)*zb)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*(cos(pitch)*cos(roll)*sin(yaw)*yb-cos(pitch)*sin(roll)*zb*sin(yaw))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);
                dgdTheta(4,4) += (2.0)*(sin(roll)*sin(pitch)*sin(yaw)*yb+cos(pitch)*xb*sin(yaw)+zb*cos(roll)*sin(pitch)*sin(yaw))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)+(2.0)*pow(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw),2.0)+(2.0)*pow(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb,2.0)-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)*yb+cos(yaw)*zb*cos(roll)*sin(pitch)+cos(pitch)*xb*cos(yaw))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch))*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)+(2.0)*pow(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb,2.0);
                dgdTheta(4,5) += -(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)+(2.0)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb);

                dgdTheta(5,0) += -(2.0)*cos(pitch)*xb*sin(yaw)+(2.0)*zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb;
                dgdTheta(5,1) += (2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+(2.0)*cos(pitch)*xb*cos(yaw);
                dgdTheta(5,2) += 0.0;
                dgdTheta(5,3) += (2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))-(2.0)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);
                dgdTheta(5,4) += -(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)+(2.0)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb);
                dgdTheta(5,5) += (2.0)*pow(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb,2.0)+(2.0)*pow((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw),2.0)-(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))+(2.0)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);

                Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                Cx.block<3,3>(setA.rows()*3+i*3,setA.rows()*3+i*3) = setBUncertainty[i];

                dgdX(i*3,0) = -2.0;
                dgdX(i*3,1) = 0.0;
                dgdX(i*3,2) = 0.0;
                dgdX(i*3,3) = -(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb+(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb;
                dgdX(i*3,4) = (2.0)*xb*cos(yaw)*sin(pitch)-(2.0)*cos(pitch)*cos(yaw)*zb*cos(roll)-(2.0)*cos(pitch)*sin(roll)*cos(yaw)*yb;
                dgdX(i*3,5) = (2.0)*cos(pitch)*xb*sin(yaw)-(2.0)*zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb;

                dgdX(i*3+1,0) = 0.0;
                dgdX(i*3+1,1) = -2.0;
                dgdX(i*3+1,2) = 0.0;
                dgdX(i*3+1,3) = (2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+(2.0)*yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw));
                dgdX(i*3+1,4) = (2.0)*xb*sin(pitch)*sin(yaw)-(2.0)*cos(pitch)*sin(roll)*sin(yaw)*yb-(2.0)*cos(pitch)*zb*cos(roll)*sin(yaw);
                dgdX(i*3+1,5) = -(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb-(2.0)*cos(pitch)*xb*cos(yaw);

                dgdX(i*3+2,0) = 0;
                dgdX(i*3+2,1) = 0;
                dgdX(i*3+2,2) = -2.0;
                dgdX(i*3+2,3) = -(2.0)*cos(pitch)*cos(roll)*yb+(2.0)*cos(pitch)*sin(roll)*zb;
                dgdX(i*3+2,4) = (2.0)*zb*cos(roll)*sin(pitch)+(2.0)*sin(roll)*sin(pitch)*yb+(2.0)*cos(pitch)*xb;
                dgdX(i*3+2,5) = 0.0;

                dgdX(setA.rows()*3+i*3,0) = (2.0)*cos(pitch)*cos(yaw);
                dgdX(setA.rows()*3+i*3,1) = (2.0)*cos(pitch)*sin(yaw);
                dgdX(setA.rows()*3+i*3,2) = -(2.0)*sin(pitch);
                dgdX(setA.rows()*3+i*3,3) = -(2.0)*cos(pitch)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*sin(yaw)+(2.0)*cos(pitch)*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)*cos(yaw)-(2.0)*sin(pitch)*(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb);
                dgdX(setA.rows()*3+i*3,4) = -(2.0)*cos(pitch)*cos(yaw)*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)-(2.0)*cos(pitch)*sin(yaw)*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))-(2.0)*cos(pitch)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)+(2.0)*sin(pitch)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*sin(yaw)+(2.0)*(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb)*sin(pitch)-(2.0)*cos(yaw)*sin(pitch)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw));
                dgdX(setA.rows()*3+i*3,5) = (2.0)*cos(pitch)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))*sin(yaw)-(2.0)*cos(pitch)*cos(yaw)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*cos(pitch)*cos(yaw)*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*cos(pitch)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))*sin(yaw);

                dgdX(setA.rows()*3+i*3+1,0) = (2.0)*sin(roll)*cos(yaw)*sin(pitch)-(2.0)*cos(roll)*sin(yaw);
                dgdX(setA.rows()*3+i*3+1,1) = (2.0)*cos(yaw)*cos(roll)+(2.0)*sin(roll)*sin(pitch)*sin(yaw);
                dgdX(setA.rows()*3+i*3+1,2) = (2.0)*cos(pitch)*sin(roll);
                dgdX(setA.rows()*3+i*3+1,3) = (2.0)*cos(pitch)*sin(roll)*(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb)+(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)-(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))+(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))+(2.0)*cos(pitch)*cos(roll)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)+(2.0)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw));
                dgdX(setA.rows()*3+i*3+1,4) = -(2.0)*cos(pitch)*sin(roll)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*sin(yaw)-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)-(2.0)*cos(pitch)*sin(roll)*(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb)-(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))+(2.0)*cos(pitch)*sin(roll)*cos(yaw)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*sin(roll)*sin(pitch)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z);
                dgdX(setA.rows()*3+i*3+1,5) = -(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))+(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw));

                dgdX(setA.rows()*3+i*3+2,0) = (2.0)*sin(roll)*sin(yaw)+(2.0)*cos(yaw)*cos(roll)*sin(pitch);
                dgdX(setA.rows()*3+i*3+2,1) = -(2.0)*sin(roll)*cos(yaw)+(2.0)*cos(roll)*sin(pitch)*sin(yaw);
                dgdX(setA.rows()*3+i*3+2,2) = (2.0)*cos(pitch)*cos(roll);
                dgdX(setA.rows()*3+i*3+2,3) = (2.0)*((cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*zb+yb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw)))*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(2.0)*cos(pitch)*sin(roll)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z)-(2.0)*(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))+(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*((sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*yb-(sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*zb)+(2.0)*cos(pitch)*cos(roll)*(cos(pitch)*cos(roll)*yb-cos(pitch)*sin(roll)*zb)+(2.0)*(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);
                dgdX(setA.rows()*3+i*3+2,4) = -(2.0)*cos(pitch)*cos(roll)*(zb*cos(roll)*sin(pitch)+sin(roll)*sin(pitch)*yb+cos(pitch)*xb)-(2.0)*cos(pitch)*cos(roll)*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)*sin(yaw)+(2.0)*(xb*sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*sin(yaw)*yb-cos(pitch)*zb*cos(roll)*sin(yaw))*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(2.0)*cos(pitch)*cos(yaw)*cos(roll)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))-(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*(xb*cos(yaw)*sin(pitch)-cos(pitch)*cos(yaw)*zb*cos(roll)-cos(pitch)*sin(roll)*cos(yaw)*yb)-(2.0)*cos(roll)*sin(pitch)*(cos(pitch)*zb*cos(roll)+cos(pitch)*sin(roll)*yb-xb*sin(pitch)-za+z);
                dgdX(setA.rows()*3+i*3+2,5) = -(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+cos(pitch)*xb*cos(yaw))*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(2.0)*((sin(roll)*cos(yaw)*sin(pitch)-cos(roll)*sin(yaw))*yb-xa+(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*zb+x+cos(pitch)*xb*cos(yaw))*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*(cos(pitch)*xb*sin(yaw)-zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))+(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb)-(2.0)*(sin(roll)*sin(yaw)+cos(yaw)*cos(roll)*sin(pitch))*(ya-y-cos(pitch)*xb*sin(yaw)+zb*(sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))-(cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw))*yb);
            }
            dgdX = k*dgdX; dgdTheta = k * dgdTheta;
            Mat66 dgdThetaInv = dgdTheta.inverse();

            //std::cout << "Cx: \n" << Cx << "\n";
            //std::cout << "dgdX: \n" << dgdX << "\n";
            //std::cout << "dgdTheta: \n" << dgdTheta << "\n";
            uncertainty = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetaInv;
            return uncertainty;
        }

        /// Compute uncertainty matrix [6x6] (x, y, z, qx, qy, qz)
        virtual const Mat66& computeUncertaintyG2O(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation) {
            Mat66 dgdTheta; dgdTheta.setZero();
            Quaternion quat(transformation.rotation());
            float_type x = transformation.matrix()(0,3); float_type y = transformation.matrix()(1,3); float_type z = transformation.matrix()(2,3);

            Eigen::MatrixXd Cx(2*3*setA.rows(),2*3*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*3*setA.rows(), 2*3*setA.rows());
            Eigen::MatrixXd dgdX(2*3*setA.rows(),6);
            float_type k = 1.0/setA.rows();
            float_type qx = quat.x(); float_type qy = quat.y(); float_type qz = quat.z(); float_type qw = quat.w();
            for (size_t i=0;i<setA.rows();i++){
                float_type xa = setA(i,0); float_type ya = setA(i,1); float_type za = setA(i,2);
                float_type xb = setB(i,0); float_type yb = setB(i,1); float_type zb = setB(i,2);

                dgdTheta(0,0) += 2.0;
                dgdTheta(0,1) += 0.0;
                dgdTheta(0,2) += 0.0;
                dgdTheta(0,3) += (4.0)*qy*yb+(4.0)*qz*zb;
                dgdTheta(0,4) += (4.0)*qw*zb+(4.0)*yb*qx-(8.0)*qy*xb;
                dgdTheta(0,5) += -(8.0)*xb*qz+(4.0)*qx*zb-(4.0)*yb*qw;

                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += 0.0;
                dgdTheta(1,3) += -(4.0)*qw*zb-(8.0)*yb*qx+(4.0)*qy*xb;
                dgdTheta(1,4) += (4.0)*qx*xb+(4.0)*qz*zb;
                dgdTheta(1,5) += (4.0)*qy*zb+(4.0)*qw*xb-(8.0)*yb*qz;

                dgdTheta(2,2) += 2.0;
                dgdTheta(2,3) += (4.0)*xb*qz-(8.0)*qx*zb+(4.0)*yb*qw;
                dgdTheta(2,4) += -(8.0)*qy*zb-(4.0)*qw*xb+(4.0)*yb*qz;
                dgdTheta(2,5) += (4.0)*qx*xb+(4.0)*qy*yb;

                dgdTheta(3,3) += ((8.0)*(za-((-(2.0)*qy*qw+(2.0)*qz*qx)*xb)-z-(zb*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0)))-(((2.0)*qz*qy+(2.0)*qw*qx)*yb))*zb)+(2.0)*pow((-(2.0)*xb*qz+(4.0)*zb*qx-(2.0)*yb*qw),2.0)-(2.0)*(-(2.0)*qz*zb-(2.0)*qy*yb)*((2.0)*qz*zb+(2.0)*qy*yb)-(8.0)*(y+zb*((2.0)*qz*qy-(2.0)*qw*qx)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb-ya+xb*((2.0)*qy*qx+(2.0)*qz*qw))*yb-(2.0)*((2.0)*zb*qw-(2.0)*xb*qy+(4.0)*yb*qx)*(-(2.0)*zb*qw+(2.0)*xb*qy-(4.0)*yb*qx);
                dgdTheta(3,4) += -(2.0)*(-(2.0)*qz*zb-(2.0)*xb*qx)*(-(2.0)*zb*qw+(2.0)*xb*qy-(4.0)*yb*qx)+(4.0)*xb*(y+zb*((2.0)*qz*qy-(2.0)*qw*qx)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb-ya+xb*((2.0)*qy*qx+(2.0)*qz*qw))+(4.0)*(xb*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))-xa+x+zb*((2.0)*qy*qw+(2.0)*qz*qx)+((2.0)*qy*qx-(2.0)*qz*qw)*yb)*yb+(2.0)*(-(2.0)*xb*qz+(4.0)*zb*qx-(2.0)*yb*qw)*(-(2.0)*qz*yb+(4.0)*zb*qy+(2.0)*xb*qw)-(2.0)*(-(2.0)*zb*qw+(4.0)*xb*qy-(2.0)*yb*qx)*((2.0)*qz*zb+(2.0)*qy*yb);
                dgdTheta(3,5) += -(2.0)*((4.0)*qz*yb-(2.0)*zb*qy-(2.0)*xb*qw)*(-(2.0)*zb*qw+(2.0)*xb*qy-(4.0)*yb*qx)+(2.0)*(-(2.0)*xb*qz+(4.0)*zb*qx-(2.0)*yb*qw)*(-(2.0)*xb*qx-(2.0)*qy*yb)+(4.0)*(xb*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))-xa+x+zb*((2.0)*qy*qw+(2.0)*qz*qx)+((2.0)*qy*qx-(2.0)*qz*qw)*yb)*zb-(2.0)*((4.0)*xb*qz-(2.0)*zb*qx+(2.0)*yb*qw)*((2.0)*qz*zb+(2.0)*qy*yb)-(4.0)*(za-(-(2.0)*qy*qw+(2.0)*qz*qx)*xb-z-zb*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))-((2.0)*qz*qy+(2.0)*qw*qx)*yb)*xb;

                dgdTheta(4,4) += -(8.0)*(z+xb*(-(2.0)*qw*qy+(2.0)*qx*qz)-za+yb*((2.0)*qz*qy+(2.0)*qw*qx)+zb*(1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0)))*zb-(2.0)*(-(2.0)*yb*qx-(2.0)*qw*zb+(4.0)*xb*qy)*((2.0)*yb*qx+(2.0)*qw*zb-(4.0)*xb*qy)-(8.0)*(xb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))-xa+((2.0)*qw*qy+(2.0)*qx*qz)*zb+(-(2.0)*qw*qz+(2.0)*qx*qy)*yb+x)*xb-(2.0)*((2.0)*yb*qz-(2.0)*qw*xb-(4.0)*zb*qy)*(-(2.0)*yb*qz+(2.0)*qw*xb+(4.0)*zb*qy)-(2.0)*((2.0)*qx*xb+(2.0)*qz*zb)*(-(2.0)*qx*xb-(2.0)*qz*zb);
                dgdTheta(4,5) += (4.0)*yb*(z+xb*(-(2.0)*qw*qy+(2.0)*qx*qz)-za+yb*((2.0)*qz*qy+(2.0)*qw*qx)+zb*(1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0)))+(4.0)*(xb*((2.0)*qw*qz+(2.0)*qx*qy)+((2.0)*qz*qy-(2.0)*qw*qx)*zb+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))-ya+y)*zb-(2.0)*((2.0)*yb*qx+(2.0)*qw*zb-(4.0)*xb*qy)*((4.0)*xb*qz-(2.0)*qx*zb+(2.0)*yb*qw)-(2.0)*(-(2.0)*yb*qy-(2.0)*qx*xb)*((2.0)*yb*qz-(2.0)*qw*xb-(4.0)*zb*qy)-(2.0)*((4.0)*yb*qz-(2.0)*qw*xb-(2.0)*zb*qy)*((2.0)*qx*xb+(2.0)*qz*zb);

                dgdTheta(5,5) += -(2.0)*((2.0)*yb*qw+(4.0)*xb*qz-(2.0)*zb*qx)*(-(2.0)*yb*qw-(4.0)*xb*qz+(2.0)*zb*qx)-(2.0)*(-(2.0)*qw*xb-(2.0)*zb*qy+(4.0)*yb*qz)*((2.0)*qw*xb+(2.0)*zb*qy-(4.0)*yb*qz)-(8.0)*xb*(zb*((2.0)*qx*qz+(2.0)*qy*qw)+x+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb+(-(2.0)*qw*qz+(2.0)*qy*qx)*yb-xa)-(2.0)*(-(2.0)*qy*yb-(2.0)*qx*xb)*((2.0)*qy*yb+(2.0)*qx*xb)-(8.0)*yb*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qz,2.0))*yb+zb*((2.0)*qy*qz-(2.0)*qw*qx)+((2.0)*qw*qz+(2.0)*qy*qx)*xb-ya+y);

                Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                Cx.block<3,3>(setA.rows()*3+i*3,setA.rows()*3+i*3) = setBUncertainty[i];

                dgdX(i*3,0) = -2.0;
                dgdX(i*3,1) = 0.0;
                dgdX(i*3,2) = 0.0;
                dgdX(i*3,3) = -(4.0)*qy*yb-(4.0)*qz*zb;
                dgdX(i*3,4) = -(4.0)*qx*yb-(4.0)*qw*zb+(8.0)*xb*qy;
                dgdX(i*3,5) = (4.0)*qw*yb+(8.0)*xb*qz-(4.0)*qx*zb;

                dgdX(i*3+1,0) = 0.0;
                dgdX(i*3+1,1) = -2.0;
                dgdX(i*3+1,2) = 0.0;
                dgdX(i*3+1,3) = (8.0)*qx*yb+(4.0)*qw*zb-(4.0)*xb*qy;
                dgdX(i*3+1,4) = -(4.0)*qx*xb-(4.0)*qz*zb;
                dgdX(i*3+1,5) = (8.0)*qz*yb-(4.0)*zb*qy-(4.0)*qw*xb;

                dgdX(i*3+2,0) = 0.0;
                dgdX(i*3+2,1) = 0.0;
                dgdX(i*3+2,2) = -2.0;
                dgdX(i*3+2,3) = -(4.0)*qw*yb-(4.0)*xb*qz+(8.0)*qx*zb;
                dgdX(i*3+2,4) = -(4.0)*qz*yb+(8.0)*zb*qy+(4.0)*qw*xb;
                dgdX(i*3+2,5) = -(4.0)*qx*xb-(4.0)*qy*yb;

                dgdX(setA.rows()*3+i*3,0) = 2.0-(4.0)*pow(qy,2.0)-(4.0)*pow(qz,2.0);
                dgdX(setA.rows()*3+i*3,1) = (4.0)*qx*qy+(4.0)*qw*qz;
                dgdX(setA.rows()*3+i*3,2) = (4.0)*qx*qz-(4.0)*qw*qy;
                dgdX(setA.rows()*3+i*3,3) = (4.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qy-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*(-(2.0)*qy*yb-(2.0)*qz*zb)-(2.0)*((4.0)*qx*yb+(2.0)*qw*zb-(2.0)*xb*qy)*((2.0)*qx*qy+(2.0)*qw*qz)-(2.0)*(-(2.0)*qw*yb-(2.0)*xb*qz+(4.0)*qx*zb)*((2.0)*qx*qz-(2.0)*qw*qy)+(4.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qz;
                dgdX(setA.rows()*3+i*3,4) = -(2.0)*((2.0)*qx*qz-(2.0)*qw*qy)*(-(2.0)*qz*yb+(4.0)*zb*qy+(2.0)*qw*xb)-(4.0)*qw*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*(-(2.0)*qx*yb-(2.0)*qw*zb+(4.0)*xb*qy)-(2.0)*(-(2.0)*qx*xb-(2.0)*qz*zb)*((2.0)*qx*qy+(2.0)*qw*qz)+(4.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qx-(8.0)*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)*qy;
                dgdX(setA.rows()*3+i*3,5) = (4.0)*qw*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)-(2.0)*((2.0)*qx*qz-(2.0)*qw*qy)*(-(2.0)*qx*xb-(2.0)*qy*yb)-(8.0)*qz*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)+(4.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qx-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*((2.0)*qw*yb+(4.0)*xb*qz-(2.0)*qx*zb)-(2.0)*((2.0)*qx*qy+(2.0)*qw*qz)*((4.0)*qz*yb-(2.0)*zb*qy-(2.0)*qw*xb);

                dgdX(setA.rows()*3+i*3+1,0) = (4.0)*qx*qy-(4.0)*qw*qz;
                dgdX(setA.rows()*3+i*3+1,1) = 2.0-(4.0)*pow(qz,2.0)-(4.0)*pow(qx,2.0);
                dgdX(setA.rows()*3+i*3+1,2) = (4.0)*qw*qx+(4.0)*qz*qy;
                dgdX(setA.rows()*3+i*3+1,3) = -(2.0)*(-(2.0)*qy*yb-(2.0)*qz*zb)*((2.0)*qx*qy-(2.0)*qw*qz)-(2.0)*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*((4.0)*qx*yb+(2.0)*qw*zb-(2.0)*xb*qy)+(4.0)*qw*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)-(8.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qx+(4.0)*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)*qy-(2.0)*((2.0)*qw*qx+(2.0)*qz*qy)*(-(2.0)*qw*yb-(2.0)*xb*qz+(4.0)*qx*zb);
                dgdX(setA.rows()*3+i*3+1,4) = (4.0)*qx*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)-(2.0)*(-(2.0)*qx*yb-(2.0)*qw*zb+(4.0)*xb*qy)*((2.0)*qx*qy-(2.0)*qw*qz)-(2.0)*((2.0)*qw*qx+(2.0)*qz*qy)*(-(2.0)*qz*yb+(4.0)*zb*qy+(2.0)*qw*xb)-(2.0)*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*(-(2.0)*qx*xb-(2.0)*qz*zb)+(4.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qz;
                dgdX(setA.rows()*3+i*3+1,5) = -(8.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qz+(4.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qy-(2.0)*((2.0)*qw*qx+(2.0)*qz*qy)*(-(2.0)*qx*xb-(2.0)*qy*yb)-(2.0)*((2.0)*qx*qy-(2.0)*qw*qz)*((2.0)*qw*yb+(4.0)*xb*qz-(2.0)*qx*zb)-(4.0)*qw*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)-(2.0)*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*((4.0)*qz*yb-(2.0)*zb*qy-(2.0)*qw*xb);

                dgdX(setA.rows()*3+i*3+2,0) = (4.0)*qx*qz+(4.0)*qw*qy;
                dgdX(setA.rows()*3+i*3+2,1) = -(4.0)*qw*qx+(4.0)*qz*qy;
                dgdX(setA.rows()*3+i*3+2,2) = 2.0-(4.0)*pow(qy,2.0)-(4.0)*pow(qx,2.0);
                dgdX(setA.rows()*3+i*3+2,3) = -(4.0)*qw*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)-(2.0)*((4.0)*qx*yb+(2.0)*qw*zb-(2.0)*xb*qy)*(-(2.0)*qw*qx+(2.0)*qz*qy)-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*(-(2.0)*qw*yb-(2.0)*xb*qz+(4.0)*qx*zb)+(4.0)*qz*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)-(2.0)*((2.0)*qx*qz+(2.0)*qw*qy)*(-(2.0)*qy*yb-(2.0)*qz*zb)-(8.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qx;
                dgdX(setA.rows()*3+i*3+2,4) = -(2.0)*(-(2.0)*qx*xb-(2.0)*qz*zb)*(-(2.0)*qw*qx+(2.0)*qz*qy)+(4.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qz-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*(-(2.0)*qz*yb+(4.0)*zb*qy+(2.0)*qw*xb)-(2.0)*((2.0)*qx*qz+(2.0)*qw*qy)*(-(2.0)*qx*yb-(2.0)*qw*zb+(4.0)*xb*qy)-(8.0)*(((2.0)*qw*qx+(2.0)*qz*qy)*yb-za+xb*((2.0)*qx*qz-(2.0)*qw*qy)+z+(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*zb)*qy+(4.0)*qw*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb);
                dgdX(setA.rows()*3+i*3+2,5) = (4.0)*qx*((1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qz,2.0))*xb-xa+((2.0)*qx*qz+(2.0)*qw*qy)*zb+x+((2.0)*qx*qy-(2.0)*qw*qz)*yb)-(2.0)*((2.0)*qx*qz+(2.0)*qw*qy)*((2.0)*qw*yb+(4.0)*xb*qz-(2.0)*qx*zb)-(2.0)*(1.0-(2.0)*pow(qy,2.0)-(2.0)*pow(qx,2.0))*(-(2.0)*qx*xb-(2.0)*qy*yb)+(4.0)*(xb*((2.0)*qx*qy+(2.0)*qw*qz)-ya+y+zb*(-(2.0)*qw*qx+(2.0)*qz*qy)+(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0))*yb)*qy-(2.0)*(-(2.0)*qw*qx+(2.0)*qz*qy)*((4.0)*qz*yb-(2.0)*zb*qy-(2.0)*qw*xb);

            }
            dgdX = k*dgdX; dgdTheta = k * dgdTheta;

            dgdTheta(1,0) = dgdTheta(0,1);

            dgdTheta(2,0) = dgdTheta(0,2);
            dgdTheta(2,1) = dgdTheta(1,2);

            dgdTheta(3,0) = dgdTheta(0,3);
            dgdTheta(3,1) = dgdTheta(1,3);
            dgdTheta(3,2) = dgdTheta(2,3);

            dgdTheta(4,0) = dgdTheta(0,4);
            dgdTheta(4,1) = dgdTheta(1,4);
            dgdTheta(4,2) = dgdTheta(2,4);
            dgdTheta(4,3) = dgdTheta(3,4);

            dgdTheta(5,0) = dgdTheta(0,5);
            dgdTheta(5,1) = dgdTheta(1,5);
            dgdTheta(5,2) = dgdTheta(2,5);
            dgdTheta(5,3) = dgdTheta(3,5);
            dgdTheta(5,4) = dgdTheta(4,5);

            Mat66 dgdThetaInv = dgdTheta.inverse();
            //Mat66 dgdThetatrans = dgdTheta.transpose();
            //Mat66 dgdThetatransInv = dgdThetatrans.inverse();

            //std::cout << "Cx: \n" << Cx << "\n";
            //std::cout << "dgdX: \n" << dgdX << "\n";
            //std::cout << "dgdTheta: \n" << dgdTheta << "\n";
            uncertainty = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetaInv;

            //uncertainty = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetatransInv;
            //std::cout << "dgdThetaInv: \n" << dgdThetaInv << "\n";
            //std::cout << "dgdThetatransInv: \n" << dgdThetatransInv << "\n";
            //getchar();

            //uncertainty = ConvertUncertaintyEuler2quat(uncertainty, transformation);
            return uncertainty;
        }

        /// Compute uncertainty matrix [6x6] (x, y, z, qx, qy, qz)
        virtual const Mat66& computeUncertaintyStrasdat(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation) {
            uncertainty.setIdentity();
            //compute average depth.
            float_type depthAv = 0;
            for (int i=0;i<setA.rows();i++){
                depthAv += sqrt(pow(setA(i,0),2.0)+pow(setA(i,1),2.0)+pow(setA(i,2),2.0));
                depthAv += sqrt(pow(setB(i,0),2.0)+pow(setB(i,1),2.0)+pow(setB(i,2),2.0));
            }
            depthAv/=2*setA.rows();
            uncertainty(0,0) = pow(transformation(0,3)/depthAv,2.0);
            uncertainty(1,1) = pow(transformation(1,3)/depthAv,2.0);
            uncertainty(2,2) = pow(transformation(2,3)/depthAv,2.0);
            return uncertainty;
        }

        protected:

            /// Estimated transformation
            Mat34 transformation;
            ///Uncertainty
            Mat66 uncertainty;
    };
};

#endif // _TRANSFORMEST_H_

