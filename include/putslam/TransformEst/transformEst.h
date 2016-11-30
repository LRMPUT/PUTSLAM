/** @file transformEst.h
 *
 * Feature-based transformation estimation interface
 *
 */

#ifndef _TRANSFORMEST_H_
#define _TRANSFORMEST_H_

#include "Defs/putslam_defs.h"
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
            double x = transformation.matrix()(0,3); double y = transformation.matrix()(1,3); double z = transformation.matrix()(2,3);

            Eigen::MatrixXd Cx(2*3*setA.rows(),2*3*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*3*setA.rows(), 2*3*setA.rows());
            Eigen::MatrixXd dgdX(2*3*setA.rows(),6);
            double k = 1.0/(double)setA.rows();
            for (Eigen::Index i=0;i<setA.rows();i++){
                double xa = setA(i,0); double ya = setA(i,1); double za = setA(i,2);
                double xb = setB(i,0); double yb = setB(i,1); double zb = setB(i,2);

                dgdTheta(0,0) += 2.0;
                dgdTheta(0,1) += 0;
                dgdTheta(0,2) += 0;
                dgdTheta(0,3) += 2*yb*sin(yaw)*sin(roll)+2*zb*sin(yaw)*cos(roll)+2*yb*cos(yaw)*sin(pitch)*cos(roll)-2*zb*cos(yaw)*sin(pitch)*sin(roll);
                dgdTheta(0,4) += -2*cos(yaw)*(-cos(pitch)*cos(roll)*zb+sin(pitch)*xb-cos(pitch)*sin(roll)*yb);
                dgdTheta(0,5) += -2*yb*sin(yaw)*sin(pitch)*sin(roll)-2*sin(yaw)*cos(pitch)*xb-2*yb*cos(yaw)*cos(roll)+2*zb*cos(yaw)*sin(roll)-2*zb*sin(yaw)*sin(pitch)*cos(roll);

                dgdTheta(1,0) += 0.0;
                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += 0.0;
                dgdTheta(1,3) += -2*yb*cos(yaw)*sin(roll)-2*zb*cos(yaw)*cos(roll)+2*yb*sin(yaw)*sin(pitch)*cos(roll)-2*zb*sin(yaw)*sin(pitch)*sin(roll);
                dgdTheta(1,4) += -2*sin(yaw)*(-cos(pitch)*cos(roll)*zb+sin(pitch)*xb-cos(pitch)*sin(roll)*yb);
                dgdTheta(1,5) += 2*cos(yaw)*cos(pitch)*xb-2*yb*sin(yaw)*cos(roll)+2*zb*sin(yaw)*sin(roll)+2*yb*cos(yaw)*sin(pitch)*sin(roll)+2*zb*cos(yaw)*sin(pitch)*cos(roll);

                dgdTheta(2,0) += 0.0;
                dgdTheta(2,1) += 0.0;
                dgdTheta(2,2) += 2.0;
                dgdTheta(2,3) += 2*cos(pitch)*(cos(roll)*yb-sin(roll)*zb);
                dgdTheta(2,4) += -2*cos(pitch)*xb-2*sin(pitch)*sin(roll)*yb-2*sin(pitch)*cos(roll)*zb;
                dgdTheta(2,5) += 0.0;

                dgdTheta(3,0) += 2*yb*sin(yaw)*sin(roll)+2*zb*sin(yaw)*cos(roll)+2*yb*cos(yaw)*sin(pitch)*cos(roll)-2*zb*cos(yaw)*sin(pitch)*sin(roll);
                dgdTheta(3,1) += -2*yb*cos(yaw)*sin(roll)-2*zb*cos(yaw)*cos(roll)+2*yb*sin(yaw)*sin(pitch)*cos(roll)-2*zb*sin(yaw)*sin(pitch)*sin(roll);
                dgdTheta(3,2) += 2*cos(pitch)*(cos(roll)*yb-sin(roll)*zb);
                dgdTheta(3,3) += -2*xa*yb*sin(yaw)*cos(roll)+2*xa*zb*sin(yaw)*sin(roll)+2*x*yb*sin(yaw)*cos(roll)-2*x*zb*sin(yaw)*sin(roll)+2*xa*yb*cos(yaw)*sin(pitch)*sin(roll)+2*xa*zb*cos(yaw)*sin(pitch)*cos(roll)-2*x*yb*cos(yaw)*sin(pitch)*sin(roll)-2*x*zb*cos(yaw)*sin(pitch)*cos(roll)+2*ya*yb*sin(yaw)*sin(pitch)*sin(roll)+2*ya*yb*cos(yaw)*cos(roll)-2*ya*zb*cos(yaw)*sin(roll)-2*y*yb*cos(yaw)*cos(roll)+2*y*zb*cos(yaw)*sin(roll)+2*za*cos(pitch)*sin(roll)*yb+2*za*cos(pitch)*cos(roll)*zb-2*z*cos(pitch)*sin(roll)*yb-2*z*cos(pitch)*cos(roll)*zb+2*ya*zb*sin(yaw)*sin(pitch)*cos(roll)-2*y*yb*sin(yaw)*sin(pitch)*sin(roll)-2*y*zb*sin(yaw)*sin(pitch)*cos(roll);
                dgdTheta(3,4) += -2*x*cos(yaw)*cos(pitch)*sin(roll)*zb-2*xa*cos(yaw)*cos(pitch)*cos(roll)*yb+2*xa*cos(yaw)*cos(pitch)*sin(roll)*zb+2*x*cos(yaw)*cos(pitch)*cos(roll)*yb+2*za*sin(pitch)*cos(roll)*yb-2*za*sin(pitch)*sin(roll)*zb-2*z*sin(pitch)*cos(roll)*yb+2*z*sin(pitch)*sin(roll)*zb-2*ya*sin(yaw)*cos(pitch)*cos(roll)*yb+2*ya*sin(yaw)*cos(pitch)*sin(roll)*zb+2*y*sin(yaw)*cos(pitch)*cos(roll)*yb-2*y*sin(yaw)*cos(pitch)*sin(roll)*zb;
                dgdTheta(3,5) += -2*x*yb*sin(yaw)*sin(pitch)*cos(roll)-2*xa*yb*cos(yaw)*sin(roll)-2*xa*zb*cos(yaw)*cos(roll)+2*x*yb*cos(yaw)*sin(roll)+2*x*zb*cos(yaw)*cos(roll)+2*xa*yb*sin(yaw)*sin(pitch)*cos(roll)-2*xa*zb*sin(yaw)*sin(pitch)*sin(roll)+2*x*zb*sin(yaw)*sin(pitch)*sin(roll)-2*ya*yb*sin(yaw)*sin(roll)-2*ya*zb*sin(yaw)*cos(roll)+2*y*yb*sin(yaw)*sin(roll)+2*y*zb*sin(yaw)*cos(roll)-2*ya*yb*cos(yaw)*sin(pitch)*cos(roll)+2*ya*zb*cos(yaw)*sin(pitch)*sin(roll)+2*y*yb*cos(yaw)*sin(pitch)*cos(roll)-2*y*zb*cos(yaw)*sin(pitch)*sin(roll);

                dgdTheta(4,0) += -2*cos(yaw)*(-cos(pitch)*cos(roll)*zb+sin(pitch)*xb-cos(pitch)*sin(roll)*yb);
                dgdTheta(4,1) += -2*sin(yaw)*(-cos(pitch)*cos(roll)*zb+sin(pitch)*xb-cos(pitch)*sin(roll)*yb);
                dgdTheta(4,2) += -2*cos(pitch)*xb-2*sin(pitch)*sin(roll)*yb-2*sin(pitch)*cos(roll)*zb;
                dgdTheta(4,3) += -2*x*cos(yaw)*cos(pitch)*sin(roll)*zb-2*xa*cos(yaw)*cos(pitch)*cos(roll)*yb+2*xa*cos(yaw)*cos(pitch)*sin(roll)*zb+2*x*cos(yaw)*cos(pitch)*cos(roll)*yb+2*za*sin(pitch)*cos(roll)*yb-2*za*sin(pitch)*sin(roll)*zb-2*z*sin(pitch)*cos(roll)*yb+2*z*sin(pitch)*sin(roll)*zb-2*ya*sin(yaw)*cos(pitch)*cos(roll)*yb+2*ya*sin(yaw)*cos(pitch)*sin(roll)*zb+2*y*sin(yaw)*cos(pitch)*cos(roll)*yb-2*y*sin(yaw)*cos(pitch)*sin(roll)*zb;
                dgdTheta(4,4) += 2*z*sin(pitch)*xb-2*za*sin(pitch)*xb-2*x*zb*cos(yaw)*sin(pitch)*cos(roll)+2*xa*cos(yaw)*cos(pitch)*xb-2*x*cos(yaw)*cos(pitch)*xb+2*xa*yb*cos(yaw)*sin(pitch)*sin(roll)+2*xa*zb*cos(yaw)*sin(pitch)*cos(roll)-2*x*yb*cos(yaw)*sin(pitch)*sin(roll)+2*ya*sin(yaw)*cos(pitch)*xb-2*y*sin(yaw)*cos(pitch)*xb+2*za*cos(pitch)*sin(roll)*yb+2*za*cos(pitch)*cos(roll)*zb-2*z*cos(pitch)*sin(roll)*yb-2*z*cos(pitch)*cos(roll)*zb+2*ya*yb*sin(yaw)*sin(pitch)*sin(roll)+2*ya*zb*sin(yaw)*sin(pitch)*cos(roll)-2*y*yb*sin(yaw)*sin(pitch)*sin(roll)-2*y*zb*sin(yaw)*sin(pitch)*cos(roll);
                dgdTheta(4,5) += -2*x*yb*sin(yaw)*cos(pitch)*sin(roll)-2*xa*sin(yaw)*sin(pitch)*xb+2*x*sin(yaw)*sin(pitch)*xb+2*xa*yb*sin(yaw)*cos(pitch)*sin(roll)+2*xa*zb*sin(yaw)*cos(pitch)*cos(roll)-2*x*zb*sin(yaw)*cos(pitch)*cos(roll)+2*ya*cos(yaw)*sin(pitch)*xb-2*y*cos(yaw)*sin(pitch)*xb-2*ya*yb*cos(yaw)*cos(pitch)*sin(roll)-2*ya*zb*cos(yaw)*cos(pitch)*cos(roll)+2*y*yb*cos(yaw)*cos(pitch)*sin(roll)+2*y*zb*cos(yaw)*cos(pitch)*cos(roll);

                dgdTheta(5,0) += -2*sin(yaw)*cos(pitch)*xb-2*yb*sin(yaw)*sin(pitch)*sin(roll)-2*yb*cos(yaw)*cos(roll)-2*zb*sin(yaw)*sin(pitch)*cos(roll)+2*zb*cos(yaw)*sin(roll);
                dgdTheta(5,1) += 2*cos(yaw)*cos(pitch)*xb+2*yb*cos(yaw)*sin(pitch)*sin(roll)-2*yb*sin(yaw)*cos(roll)+2*zb*cos(yaw)*sin(pitch)*cos(roll)+2*zb*sin(yaw)*sin(roll);
                dgdTheta(5,2) += 0.0;
                dgdTheta(5,3) += -2*xa*yb*cos(yaw)*sin(roll)-2*xa*zb*cos(yaw)*cos(roll)+2*x*yb*cos(yaw)*sin(roll)+2*x*zb*cos(yaw)*cos(roll)+2*xa*yb*sin(yaw)*sin(pitch)*cos(roll)-2*xa*zb*sin(yaw)*sin(pitch)*sin(roll)-2*x*yb*sin(yaw)*sin(pitch)*cos(roll)+2*x*zb*sin(yaw)*sin(pitch)*sin(roll)-2*ya*yb*cos(yaw)*sin(pitch)*cos(roll)-2*ya*yb*sin(yaw)*sin(roll)-2*ya*zb*sin(yaw)*cos(roll)+2*y*yb*sin(yaw)*sin(roll)+2*y*zb*sin(yaw)*cos(roll)+2*ya*zb*cos(yaw)*sin(pitch)*sin(roll)+2*y*yb*cos(yaw)*sin(pitch)*cos(roll)-2*y*zb*cos(yaw)*sin(pitch)*sin(roll);
                dgdTheta(5,4) += -2*x*yb*sin(yaw)*cos(pitch)*sin(roll)-2*xa*sin(yaw)*sin(pitch)*xb+2*x*sin(yaw)*sin(pitch)*xb+2*xa*yb*sin(yaw)*cos(pitch)*sin(roll)+2*xa*zb*sin(yaw)*cos(pitch)*cos(roll)-2*x*zb*sin(yaw)*cos(pitch)*cos(roll)+2*ya*cos(yaw)*sin(pitch)*xb-2*y*cos(yaw)*sin(pitch)*xb-2*ya*yb*cos(yaw)*cos(pitch)*sin(roll)-2*ya*zb*cos(yaw)*cos(pitch)*cos(roll)+2*y*yb*cos(yaw)*cos(pitch)*sin(roll)+2*y*zb*cos(yaw)*cos(pitch)*cos(roll);
                dgdTheta(5,5) += -2*x*yb*cos(yaw)*sin(pitch)*sin(roll)+2*xa*cos(yaw)*cos(pitch)*xb-2*xa*yb*sin(yaw)*cos(roll)+2*xa*zb*sin(yaw)*sin(roll)-2*x*cos(yaw)*cos(pitch)*xb+2*x*yb*sin(yaw)*cos(roll)-2*x*zb*sin(yaw)*sin(roll)+2*xa*yb*cos(yaw)*sin(pitch)*sin(roll)+2*xa*zb*cos(yaw)*sin(pitch)*cos(roll)-2*x*zb*cos(yaw)*sin(pitch)*cos(roll)+2*ya*sin(yaw)*cos(pitch)*xb+2*ya*yb*cos(yaw)*cos(roll)-2*ya*zb*cos(yaw)*sin(roll)-2*y*sin(yaw)*cos(pitch)*xb-2*y*yb*cos(yaw)*cos(roll)+2*y*zb*cos(yaw)*sin(roll)+2*ya*yb*sin(yaw)*sin(pitch)*sin(roll)+2*ya*zb*sin(yaw)*sin(pitch)*cos(roll)-2*y*yb*sin(yaw)*sin(pitch)*sin(roll)-2*y*zb*sin(yaw)*sin(pitch)*cos(roll);

                Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                Cx.block<3,3>(setA.rows()*3+i*3,setA.rows()*3+i*3) = setBUncertainty[i];

                dgdX(i*3,0) = -2.0;
                dgdX(i*3,1) = 0.0;
                dgdX(i*3,2) = 0.0;
                dgdX(i*3,3) = -2*yb*sin(yaw)*sin(roll)-2*zb*sin(yaw)*cos(roll)-2*yb*cos(yaw)*sin(pitch)*cos(roll)+2*zb*cos(yaw)*sin(pitch)*sin(roll);
                dgdX(i*3,4) = 2*cos(yaw)*(sin(pitch)*xb-cos(pitch)*sin(roll)*yb-cos(pitch)*cos(roll)*zb);
                dgdX(i*3,5) = 2*sin(yaw)*cos(pitch)*xb+2*yb*cos(yaw)*cos(roll)-2*zb*cos(yaw)*sin(roll)+2*yb*sin(yaw)*sin(pitch)*sin(roll)+2*zb*sin(yaw)*sin(pitch)*cos(roll);

                dgdX(i*3+1,0) = 0.0;
                dgdX(i*3+1,1) = -2.0;
                dgdX(i*3+1,2) = 0.0;
                dgdX(i*3+1,3) = -2*yb*sin(yaw)*sin(pitch)*cos(roll)+2*yb*cos(yaw)*sin(roll)+2*zb*cos(yaw)*cos(roll)+2*zb*sin(yaw)*sin(pitch)*sin(roll);
                dgdX(i*3+1,4) = 2*sin(yaw)*(sin(pitch)*xb-cos(pitch)*sin(roll)*yb-cos(pitch)*cos(roll)*zb);
                dgdX(i*3+1,5) = -2*cos(yaw)*cos(pitch)*xb+2*yb*sin(yaw)*cos(roll)-2*zb*sin(yaw)*sin(roll)-2*yb*cos(yaw)*sin(pitch)*sin(roll)-2*zb*cos(yaw)*sin(pitch)*cos(roll);

                dgdX(i*3+2,0) = 0;
                dgdX(i*3+2,1) = 0;
                dgdX(i*3+2,2) = -2.0;
                dgdX(i*3+2,3) = 2*cos(pitch)*(-cos(roll)*yb+sin(roll)*zb);
                dgdX(i*3+2,4) = 2*cos(pitch)*xb+2*sin(pitch)*sin(roll)*yb+2*sin(pitch)*cos(roll)*zb;
                dgdX(i*3+2,5) = 0.0;

                dgdX(setA.rows()*3+i*3,0) = 2*cos(yaw)*cos(pitch);
                dgdX(setA.rows()*3+i*3,1) = 2*sin(yaw)*cos(pitch);
                dgdX(setA.rows()*3+i*3,2) = -2*sin(pitch);
                dgdX(setA.rows()*3+i*3,3) = 0.0;
                dgdX(setA.rows()*3+i*3,4) = -2*z*cos(pitch)+2*za*cos(pitch)+2*xa*cos(yaw)*sin(pitch)-2*x*cos(yaw)*sin(pitch)+2*ya*sin(yaw)*sin(pitch)-2*y*sin(yaw)*sin(pitch);
                dgdX(setA.rows()*3+i*3,5) = -2*cos(pitch)*(-xa*sin(yaw)+x*sin(yaw)+ya*cos(yaw)-y*cos(yaw));

                dgdX(setA.rows()*3+i*3+1,0) = 2*cos(yaw)*sin(pitch)*sin(roll)-2*sin(yaw)*cos(roll);
                dgdX(setA.rows()*3+i*3+1,1) = 2*sin(yaw)*sin(pitch)*sin(roll)+2*cos(yaw)*cos(roll);
                dgdX(setA.rows()*3+i*3+1,2) = 2*cos(pitch)*sin(roll);
                dgdX(setA.rows()*3+i*3+1,3) = -2*xa*sin(yaw)*sin(roll)+2*x*sin(yaw)*sin(roll)-2*xa*cos(yaw)*sin(pitch)*cos(roll)+2*x*cos(yaw)*sin(pitch)*cos(roll)-2*ya*sin(yaw)*sin(pitch)*cos(roll)+2*ya*cos(yaw)*sin(roll)-2*y*cos(yaw)*sin(roll)-2*za*cos(pitch)*cos(roll)+2*z*cos(pitch)*cos(roll)+2*y*sin(yaw)*sin(pitch)*cos(roll);
                dgdX(setA.rows()*3+i*3+1,4) = 2*sin(roll)*(-xa*cos(yaw)*cos(pitch)+x*cos(yaw)*cos(pitch)+za*sin(pitch)-z*sin(pitch)-ya*sin(yaw)*cos(pitch)+y*sin(yaw)*cos(pitch));
                dgdX(setA.rows()*3+i*3+1,5) = -2*x*sin(yaw)*sin(pitch)*sin(roll)+2*xa*cos(yaw)*cos(roll)-2*x*cos(yaw)*cos(roll)+2*xa*sin(yaw)*sin(pitch)*sin(roll)+2*ya*sin(yaw)*cos(roll)-2*y*sin(yaw)*cos(roll)-2*ya*cos(yaw)*sin(pitch)*sin(roll)+2*y*cos(yaw)*sin(pitch)*sin(roll);

                dgdX(setA.rows()*3+i*3+2,0) = 2*cos(yaw)*sin(pitch)*cos(roll)+2*sin(yaw)*sin(roll);
                dgdX(setA.rows()*3+i*3+2,1) = 2*sin(yaw)*sin(pitch)*cos(roll)-2*cos(yaw)*sin(roll);
                dgdX(setA.rows()*3+i*3+2,2) = 2*cos(pitch)*cos(roll);
                dgdX(setA.rows()*3+i*3+2,3) = -2*xa*sin(yaw)*cos(roll)+2*x*sin(yaw)*cos(roll)+2*xa*cos(yaw)*sin(pitch)*sin(roll)-2*x*cos(yaw)*sin(pitch)*sin(roll)+2*ya*cos(yaw)*cos(roll)-2*y*cos(yaw)*cos(roll)+2*za*cos(pitch)*sin(roll)-2*z*cos(pitch)*sin(roll)+2*ya*sin(yaw)*sin(pitch)*sin(roll)-2*y*sin(yaw)*sin(pitch)*sin(roll);
                dgdX(setA.rows()*3+i*3+2,4) = 2*cos(roll)*(-xa*cos(yaw)*cos(pitch)+x*cos(yaw)*cos(pitch)+za*sin(pitch)-z*sin(pitch)-ya*sin(yaw)*cos(pitch)+y*sin(yaw)*cos(pitch));
                dgdX(setA.rows()*3+i*3+2,5) = -2*xa*cos(yaw)*sin(roll)+2*x*cos(yaw)*sin(roll)+2*xa*sin(yaw)*sin(pitch)*cos(roll)-2*x*sin(yaw)*sin(pitch)*cos(roll)-2*ya*sin(yaw)*sin(roll)+2*y*sin(yaw)*sin(roll)-2*ya*cos(yaw)*sin(pitch)*cos(roll)+2*y*cos(yaw)*sin(pitch)*cos(roll);
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
            double x = transformation.matrix()(0,3); double y = transformation.matrix()(1,3); double z = transformation.matrix()(2,3);

            Eigen::MatrixXd Cx(2*3*setA.rows(),2*3*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*3*setA.rows(), 2*3*setA.rows());
            Eigen::MatrixXd dgdX(2*3*setA.rows(),6);
            double k = 1.0/(double)setA.rows();
            double qx = quat.x(); double qy = quat.y(); double qz = quat.z(); double qw = quat.w();
            for (Eigen::Index i=0;i<setA.rows();i++){
                double xa = setA(i,0); double ya = setA(i,1); double za = setA(i,2);
                double xb = setB(i,0); double yb = setB(i,1); double zb = setB(i,2);

                dgdTheta(0,0) += 2.0;
                dgdTheta(0,1) += 0.0;
                dgdTheta(0,2) += 0.0;
                dgdTheta(0,3) += 4*qz*zb+4*qy*yb;
                dgdTheta(0,4) += -8*qy*xb+4*qx*yb+4*qw*zb;
                dgdTheta(0,5) += -4*qw*yb+4*qx*zb-8*qz*xb;

                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += 0.0;
                dgdTheta(1,3) += -8*qx*yb-4*qw*zb+4*qy*xb;
                dgdTheta(1,4) += 4*qx*xb+4*qz*zb;
                dgdTheta(1,5) += 4*qw*xb+4*qy*zb-8*qz*yb;

                dgdTheta(2,2) += 2.0;
                dgdTheta(2,3) += 4*qz*xb+4*qw*yb-8*qx*zb;
                dgdTheta(2,4) += -4*qw*xb+4*qz*yb-8*qy*zb;
                dgdTheta(2,5) += 4*qx*xb+4*qy*yb;

                dgdTheta(3,3) += -8*y*yb+8*pow(zb,2.0)*pow(qz,2.0)+16*pow(yb,2.0)*pow(qz,2.0)+8*pow(zb,2.0)*pow(qw,2.0)-8*z*zb+8*pow(xb,2.0)*pow(qy,2.0)+8*ya*yb+8*pow(yb,2.0)*pow(qy,2.0)+8*pow(xb,2.0)*pow(qz,2.0)+8*pow(yb,2.0)*pow(qw,2.0)+16*pow(zb,2.0)*pow(qy,2.0)-8*pow(yb,2.0)-16*yb*qy*qz*zb-48*xb*qx*qy*yb-48*xb*qx*qz*zb+48*pow(yb,2.0)*pow(qx,2.0)-8*pow(zb,2.0)+48*pow(zb,2.0)*pow(qx,2.0)+8*za*zb;
                dgdTheta(3,4) += -4*xa*yb+4*x*yb+4*y*xb-24*xb*pow(qy,2.0)*yb-8*xb*pow(qz,2.0)*yb-16*zb*qx*qz*yb-8*yb*pow(qw,2.0)*xb+8*xb*yb+16*pow(xb,2.0)*qx*qy-24*yb*pow(qx,2.0)*xb-4*ya*xb+16*pow(yb,2.0)*qx*qy-16*zb*qz*qy*xb+32*pow(zb,2.0)*qx*qy;
                dgdTheta(3,5) += 4*x*zb-16*yb*qy*qz*xb-8*xb*pow(qy,2.0)*zb-24*xb*pow(qz,2.0)*zb-16*yb*qx*qy*zb-8*zb*pow(qw,2.0)*xb+16*pow(zb,2.0)*qx*qz-4*xa*zb+8*xb*zb+32*pow(yb,2.0)*qx*qz+16*pow(xb,2.0)*qx*qz-24*zb*pow(qx,2.0)*xb-4*za*xb+4*z*xb;

                dgdTheta(4,4) += -8*x*xb+8*pow(zb,2.0)*pow(qw,2.0)+16*pow(xb,2.0)*pow(qz,2.0)-48*xb*qx*qy*yb-48*yb*qy*qz*zb+8*xa*xb+8*pow(xb,2.0)*pow(qx,2.0)+8*pow(zb,2.0)*pow(qz,2.0)-8*pow(xb,2.0)+48*pow(xb,2.0)*pow(qy,2.0)-8*pow(zb,2.0)+48*pow(zb,2.0)*pow(qy,2.0)+8*pow(yb,2.0)*pow(qx,2.0)-16*xb*qx*qz*zb+8*pow(xb,2.0)*pow(qw,2.0)+8*pow(yb,2.0)*pow(qz,2.0)+16*pow(zb,2.0)*pow(qx,2.0)+8*za*zb-8*z*zb;
                dgdTheta(4,5) += 4*y*zb-16*yb*qx*qz*xb-16*xb*qy*qx*zb-8*yb*pow(qx,2.0)*zb-8*zb*pow(qw,2.0)*yb-24*yb*pow(qz,2.0)*zb+32*pow(xb,2.0)*qy*qz+16*pow(zb,2.0)*qy*qz-4*ya*zb+8*yb*zb+16*pow(yb,2.0)*qy*qz-24*zb*pow(qy,2.0)*yb-4*za*yb+4*z*yb;

                dgdTheta(5,5) += -8*x*xb-16*xb*qx*qy*yb-48*xb*qx*qz*zb-48*yb*qy*qz*zb+16*pow(xb,2.0)*pow(qy,2.0)+8*pow(yb,2.0)*pow(qw,2.0)+8*pow(zb,2.0)*pow(qx,2.0)+8*xa*xb-8*pow(xb,2.0)+48*pow(xb,2.0)*pow(qz,2.0)-8*pow(yb,2.0)+48*pow(yb,2.0)*pow(qz,2.0)+8*pow(xb,2.0)*pow(qw,2.0)+16*pow(yb,2.0)*pow(qx,2.0)+8*pow(zb,2.0)*pow(qy,2.0)+8*ya*yb-8*y*yb+8*pow(xb,2.0)*pow(qx,2.0)+8*pow(yb,2.0)*pow(qy,2.0);

                Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                Cx.block<3,3>(setA.rows()*3+i*3,setA.rows()*3+i*3) = setBUncertainty[i];

                dgdX(i*3,0) = -2.0;
                dgdX(i*3,1) = 0.0;
                dgdX(i*3,2) = 0.0;
                dgdX(i*3,3) = -4*qy*yb-4*qz*zb;
                dgdX(i*3,4) = -4*qx*yb+8*qy*xb-4*qw*zb;
                dgdX(i*3,5) = 4*qw*yb+8*qz*xb-4*qx*zb;

                dgdX(i*3+1,0) = 0.0;
                dgdX(i*3+1,1) = -2.0;
                dgdX(i*3+1,2) = 0.0;
                dgdX(i*3+1,3) = -4*qy*xb+4*qw*zb+8*qx*yb;
                dgdX(i*3+1,4) = -4*qz*zb-4*qx*xb;
                dgdX(i*3+1,5) = -4*qw*xb+8*qz*yb-4*qy*zb;

                dgdX(i*3+2,0) = 0.0;
                dgdX(i*3+2,1) = 0.0;
                dgdX(i*3+2,2) = -2.0;
                dgdX(i*3+2,3) = -4*qw*yb-4*qz*xb+8*qx*zb;
                dgdX(i*3+2,4) = 4*qw*xb-4*qz*yb+8*qy*zb;
                dgdX(i*3+2,5) = -4*qx*xb-4*qy*yb;

                dgdX(setA.rows()*3+i*3,0) = 2-4*pow(qy,2.0)-4*pow(qz,2.0);
                dgdX(setA.rows()*3+i*3,1) = 4*qx*qy+4*qz*qw;
                dgdX(setA.rows()*3+i*3,2) = 4*qx*qz-4*qy*qw;
                dgdX(setA.rows()*3+i*3,3) = 8*qz*zb+4*z*qz+8*qy*yb-4*ya*qy-8*pow(qy,3.0)*yb+4*y*qy+16*xb*qx*pow(qy,2.0)-8*pow(qz,3.0)*zb-8*pow(qy,2.0)*qz*zb-8*pow(qz,2.0)*qy*yb+16*xb*qx*pow(qz,2.0)-24*pow(qx,2.0)*qy*yb-8*qz*pow(qw,2.0)*zb-24*pow(qx,2.0)*qz*zb-8*qy*pow(qw,2.0)*yb-4*za*qz;
                dgdX(setA.rows()*3+i*3,4) = -8*x*qy-4*z*qw+32*xb*pow(qz,2.0)*qy+4*y*qx-24*pow(qy,2.0)*qx*yb-8*pow(qz,2.0)*qx*yb-8*yb*qx*pow(qw,2.0)+8*xa*qy+8*qx*yb+16*xb*pow(qx,2.0)*qy-8*yb*pow(qx,3.0)-4*ya*qx-16*qy*xb+32*xb*pow(qy,3.0)-16*zb*qx*qz*qy+16*xb*qy*pow(qw,2.0)+4*za*qw;
                dgdX(setA.rows()*3+i*3,5) = 4*y*qw-8*x*qz-16*yb*qx*qy*qz-8*pow(qy,2.0)*qx*zb-24*pow(qz,2.0)*qx*zb-8*zb*qx*pow(qw,2.0)+32*xb*pow(qy,2.0)*qz+8*xa*qz+8*qx*zb-16*qz*xb+32*xb*pow(qz,3.0)+16*xb*qz*pow(qw,2.0)-4*ya*qw+16*xb*pow(qx,2.0)*qz-8*zb*pow(qx,3.0)-4*za*qx+4*z*qx;

                dgdX(setA.rows()*3+i*3+1,0) = 4*qx*qy-4*qz*qw;
                dgdX(setA.rows()*3+i*3+1,1) = 2-4*pow(qx,2.0)-4*pow(qz,2.0);
                dgdX(setA.rows()*3+i*3+1,2) = 4*qy*qz+4*qx*qw;
                dgdX(setA.rows()*3+i*3+1,3) = -8*y*qx+32*pow(qz,2.0)*qx*yb+4*z*qw-4*xa*qy+8*qy*xb+4*x*qy-8*xb*pow(qy,3.0)-4*za*qw+8*ya*qx+16*pow(qy,2.0)*qx*yb-8*xb*pow(qz,2.0)*qy+16*yb*qx*pow(qw,2.0)-16*qx*yb-16*zb*qx*qz*qy-24*xb*pow(qx,2.0)*qy-8*xb*qy*pow(qw,2.0)+32*yb*pow(qx,3.0);
                dgdX(setA.rows()*3+i*3+1,4) = -4*xa*qx+4*x*qx-24*xb*qx*pow(qy,2.0)-8*xb*qx*pow(qz,2.0)-8*qz*pow(qw,2.0)*zb-8*pow(qx,2.0)*qz*zb-24*pow(qy,2.0)*qz*zb-8*qx*pow(qw,2.0)*xb+8*qx*xb-8*pow(qx,3.0)*xb-8*pow(qz,3.0)*zb+16*pow(qx,2.0)*qy*yb+8*qz*zb+16*pow(qz,2.0)*qy*yb-4*za*qz+4*z*qz;
                dgdX(setA.rows()*3+i*3+1,5) = 4*xa*qw-4*x*qw-16*qx*qy*qz*xb-8*pow(qx,2.0)*qy*zb-8*zb*qy*pow(qw,2.0)-24*pow(qz,2.0)*qy*zb+16*yb*qz*pow(qw,2.0)-16*qz*yb+32*yb*pow(qz,3.0)+32*yb*pow(qx,2.0)*qz+8*ya*qz+8*qy*zb-8*y*qz+16*yb*pow(qy,2.0)*qz-8*zb*pow(qy,3.0)-4*za*qy+4*z*qy;

                dgdX(setA.rows()*3+i*3+2,0) = 4*qx*qz+4*qy*qw;
                dgdX(setA.rows()*3+i*3+2,1) = 4*qy*qz-4*qx*qw;
                dgdX(setA.rows()*3+i*3+2,2) = 2-4*pow(qx,2.0)-4*pow(qy,2.0);
                dgdX(setA.rows()*3+i*3+2,3) = 8*qz*xb+4*x*qz+16*pow(qz,2.0)*qx*zb-4*y*qw-4*xa*qz+16*zb*qx*pow(qw,2.0)-8*z*qx+4*ya*qw-8*xb*pow(qz,3.0)-8*xb*pow(qy,2.0)*qz+32*pow(qy,2.0)*qx*zb-16*yb*qx*qy*qz-8*xb*qz*pow(qw,2.0)-24*xb*pow(qx,2.0)*qz-16*qx*zb+32*zb*pow(qx,3.0)+8*za*qx;
                dgdX(setA.rows()*3+i*3+2,4) = 16*zb*qy*pow(qw,2.0)+4*x*qw-4*ya*qz-8*yb*qz*pow(qw,2.0)-8*yb*pow(qx,2.0)*qz-24*yb*pow(qy,2.0)*qz-4*xa*qw-8*yb*pow(qz,3.0)+16*pow(qz,2.0)*qy*zb-16*qy*zb+32*zb*pow(qy,3.0)-16*qx*qy*qz*xb+8*qz*yb+4*y*qz+32*pow(qx,2.0)*qy*zb+8*za*qy-8*z*qy;
                dgdX(setA.rows()*3+i*3+2,5) = 4*y*qy+4*x*qx-8*xb*qx*pow(qy,2.0)-24*xb*qx*pow(qz,2.0)-8*pow(qx,2.0)*qy*yb-8*qy*pow(qw,2.0)*yb-24*pow(qz,2.0)*qy*yb-8*qx*pow(qw,2.0)*xb+16*pow(qx,2.0)*qz*zb-4*xa*qx+8*qx*xb+16*pow(qy,2.0)*qz*zb-4*ya*qy+8*qy*yb-8*pow(qx,3.0)*xb-8*pow(qy,3.0)*yb;

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
        virtual const Mat33 computeUncertainty2D(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation) {
            Mat33 dgdTheta; dgdTheta.setZero();
            Mat33 uncert;
            Quaternion q(transformation.rotation());
            const double& q0 = q.w();
            const double& q1 = q.x();
            const double& q2 = q.y();
            const double& q3 = q.z();
//            double roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)); // r32/r33
//            double pitch = asin(2*(q0*q2-q3*q1));
            double yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
            double theta = yaw;

            double x = transformation.matrix()(0,3); double y = transformation.matrix()(1,3); //double z = transformation.matrix()(2,3);
std::cout << "x, y, theta :\n" << x << " , " << y << "," << theta << "\n";
            Eigen::MatrixXd Cx(2*2*setA.rows(),2*2*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*2*setA.rows(), 2*2*setA.rows());
            Eigen::MatrixXd dgdX(2*2*setA.rows(),3);
            double k = 1.0/(double)setA.rows();
            for (Eigen::Index i=0;i<setA.rows();i++){
                double xa = setA(i,0); double ya = setA(i,1);
                double xb = setB(i,0); double yb = setB(i,1);

                dgdTheta(0,0) += 2.0;
                dgdTheta(0,1) += 0.0;
                dgdTheta(0,2) += -(2.0)*sin(theta)*xb-(2.0)*yb*cos(theta);

                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += (2.0)*cos(theta)*xb-(2.0)*sin(theta)*yb;

                dgdTheta(2,2) += (2.0)*(ya-sin(theta)*xb-yb*cos(theta)-y)*(sin(theta)*xb+yb*cos(theta))+(2.0)*pow(cos(theta)*xb-sin(theta)*yb,2.0)+(2.0)*pow(sin(theta)*xb+yb*cos(theta),2.0)-(2.0)*(x-xa+cos(theta)*xb-sin(theta)*yb)*(cos(theta)*xb-sin(theta)*yb);

                Cx(i*2,i*2) = setAUncertainty[i].matrix()(0,0); Cx(i*2,i*2+1) = setAUncertainty[i].matrix()(0,1);
                Cx(i*2+1,i*2) = setAUncertainty[i].matrix()(1,0); Cx(i*2+1,i*2+1) = setAUncertainty[i].matrix()(1,1);
                Cx(setA.rows()*2+i*2,setA.rows()*2+i*2) = setBUncertainty[i].matrix()(0,0); Cx(setA.rows()*2+i*2,setA.rows()*2+i*2+1) = setBUncertainty[i].matrix()(0,1);
                Cx(setA.rows()*2+i*2+1,setA.rows()*2+i*2) = setBUncertainty[i].matrix()(1,0); Cx(setA.rows()*2+i*2+1,setA.rows()*2+i*2+1) = setBUncertainty[i].matrix()(1,1);

                dgdX(i*2,0) = -2.0;
                dgdX(i*2,1) = 0.0;
                dgdX(i*2,2) = (2.0)*sin(theta)*xb+(2.0)*yb*cos(theta);

                dgdX(i*2+1,0) = 0.0;
                dgdX(i*2+1,1) = -2.0;
                dgdX(i*2+1,2) = -(2.0)*cos(theta)*xb+(2.0)*sin(theta)*yb;

                dgdX(setA.rows()*2+i*2,0) = (2.0)*cos(theta);
                dgdX(setA.rows()*2+i*2,1) = (2.0)*sin(theta);
                dgdX(setA.rows()*2+i*2,2) = -(2.0)*sin(theta)*(x-xa+cos(theta)*xb-sin(theta)*yb)-(2.0)*cos(theta)*(sin(theta)*xb+yb*cos(theta))-(2.0)*(ya-sin(theta)*xb-yb*cos(theta)-y)*cos(theta)+(2.0)*sin(theta)*(cos(theta)*xb-sin(theta)*yb);

                dgdX(setA.rows()*2+i*2+1,0) = -(2.0)*sin(theta);
                dgdX(setA.rows()*2+i*2+1,1) = (2.0)*cos(theta);
                dgdX(setA.rows()*2+i*2+1,2) = (2.0)*sin(theta)*(ya-sin(theta)*xb-yb*cos(theta)-y)+(2.0)*cos(theta)*(cos(theta)*xb-sin(theta)*yb)-(2.0)*(x-xa+cos(theta)*xb-sin(theta)*yb)*cos(theta)+(2.0)*sin(theta)*(sin(theta)*xb+yb*cos(theta));

            }
            dgdX = k*dgdX; dgdTheta = k * dgdTheta;

            dgdTheta(1,0) = dgdTheta(0,1);

            dgdTheta(2,0) = dgdTheta(0,2);
            dgdTheta(2,1) = dgdTheta(1,2);

            Mat33 dgdThetaInv = dgdTheta.inverse();
            uncert = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetaInv;

            return uncert;
        }

        /// Compute uncertainty matrix [6x6] (x, y, z, qx, qy, qz)
        virtual const Mat66& computeUncertaintyStrasdat(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB, Mat34& transformation) {
            uncertainty.setIdentity();
            //compute average depth.
            double depthAv = 0;
            for (int i=0;i<setA.rows();i++){
                depthAv += sqrt(pow(setA(i,0),2.0)+pow(setA(i,1),2.0)+pow(setA(i,2),2.0));
                depthAv += sqrt(pow(setB(i,0),2.0)+pow(setB(i,1),2.0)+pow(setB(i,2),2.0));
            }
            depthAv/=2*(double)setA.rows();
            uncertainty(0,0) = pow(transformation(0,3)/depthAv,2.0);
            uncertainty(1,1) = pow(transformation(1,3)/depthAv,2.0);
            uncertainty(2,2) = pow(transformation(2,3)/depthAv,2.0);
            return uncertainty;
        }

        /// convert [x y z fi psi theta] uncertainty matrix to [x y z qx qy qz]
        virtual const Mat66& ConvertUncertaintyEuler2quat(const Mat66& _uncertainty, const Mat34& transformation) {
            double fi = atan2(transformation.matrix()(1,0), transformation.matrix()(0,0));
            double psi = -asin(transformation.matrix()(2,0));
            double theta = atan2(transformation.matrix()(2,1), transformation.matrix()(2,2));
//            double x = transformation.matrix()(0,3); double y = transformation.matrix()(1,3); double z = transformation.matrix()(2,3);
            Mat66 J;
            J(0,0) = 1; J(0,1) = 0; J(0,2) = 0; J(0,3) = 0; J(0,4) = 0; J(0,5) = 0;
            J(1,0) = 0; J(1,1) = 1; J(1,2) = 0; J(1,3) = 0; J(1,4) = 0; J(1,5) = 0;
            J(2,0) = 0; J(2,1) = 0; J(2,2) = 1; J(2,3) = 0; J(2,4) = 0; J(2,5) = 0;
            J(3,0) = 0; J(3,1) = 0; J(3,2) = 0;
            J(3,3) = -(0.5)*cos((0.5)*psi)*sin((0.5)*fi)*sin((0.5)*theta)-(0.5)*cos((0.5)*theta)*sin((0.5)*psi)*cos((0.5)*fi);
            J(3,4) = -(0.5)*cos((0.5)*psi)*sin((0.5)*fi)*cos((0.5)*theta)-(0.5)*sin((0.5)*psi)*sin((0.5)*theta)*cos((0.5)*fi);
            J(3,5) = (0.5)*cos((0.5)*psi)*cos((0.5)*theta)*cos((0.5)*fi)+(0.5)*sin((0.5)*fi)*sin((0.5)*psi)*sin((0.5)*theta);

            J(4,0) = 0; J(4,1) = 0; J(4,2) = 0;
            J(4,3) = (0.5)*cos((0.5)*psi)*sin((0.5)*theta)*cos((0.5)*fi)-(0.5)*sin((0.5)*fi)*cos((0.5)*theta)*sin((0.5)*psi);
            J(4,4) = (0.5)*cos((0.5)*psi)*cos((0.5)*theta)*cos((0.5)*fi)-(0.5)*sin((0.5)*fi)*sin((0.5)*psi)*sin((0.5)*theta);
            J(4,5) = (0.5)*cos((0.5)*psi)*sin((0.5)*fi)*cos((0.5)*theta)-(0.5)*sin((0.5)*psi)*sin((0.5)*theta)*cos((0.5)*fi);

            J(5,0) = 0; J(5,1) = 0; J(5,2) = 0;
            J(5,3) = (0.5)*cos((0.5)*psi)*cos((0.5)*theta)*cos((0.5)*fi)+(0.5)*sin((0.5)*fi)*sin((0.5)*psi)*sin((0.5)*theta);
            J(5,4) = -(0.5)*cos((0.5)*psi)*sin((0.5)*theta)*cos((0.5)*fi)-(0.5)*sin((0.5)*fi)*cos((0.5)*theta)*sin((0.5)*psi);
            J(5,5) = -(0.5)*cos((0.5)*psi)*sin((0.5)*fi)*sin((0.5)*theta)-(0.5)*cos((0.5)*theta)*sin((0.5)*psi)*cos((0.5)*fi);
            uncertainty = J*_uncertainty*J.transpose();
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

