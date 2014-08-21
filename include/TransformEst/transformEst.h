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
            //Eigen::Matrix<double,3,1> euler = transformation.rotation().eulerAngles(2, 1, 0);//z-y-x convention
            //double fi = euler(0); double psi = euler(1); double theta = euler(2);
            //std::cout << "euler: " << "fi: " << fi << ", psi: " << psi << ",theta: " << theta <<"\n";
            double fi = atan2(transformation.matrix()(1,0), transformation.matrix()(0,0));
            double psi = -asin(transformation.matrix()(2,0));
            double theta = atan2(transformation.matrix()(2,1), transformation.matrix()(2,2));
            //std::cout << "euler: " << "fi: " << fi << ", psi: " << psi << ", theta: " << theta <<"\n";
            float_type x = transformation.matrix()(0,3); float_type y = transformation.matrix()(1,3); float_type z = transformation.matrix()(2,3);
            //std::cout << "transl: " << "x: " << x << ", y: " << y << ", z: " << z <<"\n";
            Eigen::MatrixXd Cx(2*3*setA.rows(),2*3*setA.rows());
            Cx = Eigen::ArrayXXd::Zero(2*3*setA.rows(), 2*3*setA.rows());
            Eigen::MatrixXd dgdX(2*3*setA.rows(),6);
            float_type k = 1.0/setA.rows();
            for (size_t i=0;i<setA.rows();i++){
                float_type xa = setA(i,0); float_type ya = setA(i,1); float_type za = setA(i,2);
                float_type xb = setB(i,0); float_type yb = setB(i,1); float_type zb = setB(i,2);
                float_type sinfi = sin(fi); float_type cosfi = cos(fi);
                float_type sinpsi = sin(psi); float_type cospsi = cos(psi);
                float_type sintheta = sin(theta); float_type costheta = cos(theta);

                dgdTheta(0,0) += (2.0)*pow(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)),2.0)+(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))-(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)+(2.0)*pow(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)),2.0);
                dgdTheta(0,1) += -(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))-(2.0)*(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))-(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))+(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x);
                dgdTheta(0,2) += -(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))+(2.0)*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)-(2.0)*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))*(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))-(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)));
                dgdTheta(0,3) += -(2.0)*xb*cos(psi)*sin(fi)-(2.0)*zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-(2.0)*yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi));
                dgdTheta(0,4) += (2.0)*cos(fi)*xb*cos(psi)+(2.0)*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-(2.0)*yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta));
                dgdTheta(0,5) += 0;

                dgdTheta(1,0) += -(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))-(2.0)*(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))-(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))+(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x);
                dgdTheta(1,1) += (2.0)*pow(yb*sin(psi)*sin(theta)+xb*cos(psi)+sin(psi)*zb*cos(theta),2.0)+(2.0)*pow(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi),2.0)-(2.0)*(cos(fi)*xb*cos(psi)+sin(psi)*cos(fi)*zb*cos(theta)+yb*sin(psi)*cos(fi)*sin(theta))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)-(2.0)*(sin(psi)*xb-yb*sin(theta)*cos(psi)+za-cos(psi)*zb*cos(theta)-z)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta))+(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(xb*cos(psi)*sin(fi)+sin(psi)*sin(fi)*zb*cos(theta)+yb*sin(psi)*sin(theta)*sin(fi))+(2.0)*pow(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi),2.0);
                dgdTheta(1,2) += -(2.0)*(yb*sin(psi)*sin(theta)+xb*cos(psi)+sin(psi)*zb*cos(theta))*(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb)-(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(yb*cos(psi)*sin(fi)*cos(theta)-sin(theta)*cos(psi)*sin(fi)*zb)+(2.0)*(sin(psi)*xb-yb*sin(theta)*cos(psi)+za-cos(psi)*zb*cos(theta)-z)*(yb*sin(psi)*cos(theta)-sin(psi)*sin(theta)*zb)+(2.0)*(yb*cos(fi)*cos(psi)*cos(theta)-cos(fi)*sin(theta)*cos(psi)*zb)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)+(2.0)*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))+(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)));
                dgdTheta(1,3) += (2.0)*cos(fi)*cos(psi)*zb*cos(theta)-(2.0)*sin(psi)*cos(fi)*xb+(2.0)*yb*cos(fi)*sin(theta)*cos(psi);
                dgdTheta(1,4) += -(2.0)*sin(psi)*xb*sin(fi)+(2.0)*cos(psi)*sin(fi)*zb*cos(theta)+(2.0)*yb*sin(theta)*cos(psi)*sin(fi);
                dgdTheta(1,5) += -(2.0)*yb*sin(psi)*sin(theta)-(2.0)*xb*cos(psi)-(2.0)*sin(psi)*zb*cos(theta);

                dgdTheta(2,0) += -(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))+(2.0)*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)-(2.0)*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))*(xb*cos(psi)*sin(fi)+zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))-(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)));
                dgdTheta(2,1) += -(2.0)*(yb*sin(psi)*sin(theta)+xb*cos(psi)+sin(psi)*zb*cos(theta))*(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb)-(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(yb*cos(psi)*sin(fi)*cos(theta)-sin(theta)*cos(psi)*sin(fi)*zb)+(2.0)*(sin(psi)*xb-yb*sin(theta)*cos(psi)+za-cos(psi)*zb*cos(theta)-z)*(yb*sin(psi)*cos(theta)-sin(psi)*sin(theta)*zb)+(2.0)*(yb*cos(fi)*cos(psi)*cos(theta)-cos(fi)*sin(theta)*cos(psi)*zb)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)+(2.0)*((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)))*(cos(fi)*cos(psi)*zb*cos(theta)-sin(psi)*cos(fi)*xb+yb*cos(fi)*sin(theta)*cos(psi))+(2.0)*(sin(psi)*xb*sin(fi)-cos(psi)*sin(fi)*zb*cos(theta)-yb*sin(theta)*cos(psi)*sin(fi))*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)));
                dgdTheta(2,2) += (2.0)*pow(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb,2.0)+(2.0)*pow(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta)),2.0)-(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))+x)*((sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta)))+(2.0)*(ya-y-xb*cos(psi)*sin(fi)-zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))+yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))+(2.0)*pow((sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta)),2.0)+(2.0)*(sin(psi)*xb-yb*sin(theta)*cos(psi)+za-cos(psi)*zb*cos(theta)-z)*(yb*sin(theta)*cos(psi)+cos(psi)*zb*cos(theta));
                dgdTheta(2,3) += (2.0)*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+(2.0)*yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta));
                dgdTheta(2,4) += -(2.0)*zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+(2.0)*yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta));
                dgdTheta(2,5) += (2.0)*yb*cos(psi)*cos(theta)-(2.0)*sin(theta)*cos(psi)*zb;

                dgdTheta(3,0) += -(2.0)*xb*cos(psi)*sin(fi)-(2.0)*zb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta))-(2.0)*yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi));
                dgdTheta(3,1) += (2.0)*cos(fi)*cos(psi)*zb*cos(theta)-(2.0)*sin(psi)*cos(fi)*xb+(2.0)*yb*cos(fi)*sin(theta)*cos(psi);
                dgdTheta(3,2) += (2.0)*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta))*zb+(2.0)*yb*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta));
                dgdTheta(3,3) += 2.0;
                dgdTheta(3,4) += 0;
                dgdTheta(3,5) += 0;

                dgdTheta(4,0) += (2.0)*cos(fi)*xb*cos(psi)+(2.0)*(sin(theta)*sin(fi)+sin(psi)*cos(fi)*cos(theta))*zb-(2.0)*yb*(sin(fi)*cos(theta)-sin(psi)*cos(fi)*sin(theta));
                dgdTheta(4,1) += -(2.0)*sin(psi)*xb*sin(fi)+(2.0)*cos(psi)*sin(fi)*zb*cos(theta)+(2.0)*yb*sin(theta)*cos(psi)*sin(fi);
                dgdTheta(4,2) += -(2.0)*zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+(2.0)*yb*(sin(psi)*sin(fi)*cos(theta)-cos(fi)*sin(theta));
                dgdTheta(4,3) += 0;
                dgdTheta(4,4) += 2.0;
                dgdTheta(4,5) += 0;

                dgdTheta(5,0) += 0;
                dgdTheta(5,1) += -(2.0)*yb*sin(psi)*sin(theta)-(2.0)*xb*cos(psi)-(2.0)*sin(psi)*zb*cos(theta);
                dgdTheta(5,2) += (2.0)*yb*cos(psi)*cos(theta)-(2.0)*sin(theta)*cos(psi)*zb;
                dgdTheta(5,3) += 0;
                dgdTheta(5,4) += 0;
                dgdTheta(5,5) += 2.0;

                Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                Cx.block<3,3>(setA.rows()*3+i*3,setA.rows()*3+i*3) = setBUncertainty[i];

                dgdX(i*3,0) = -(2.0)*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+(2.0)*xb*cos(psi)*sin(fi)+(2.0)*yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi));
                dgdX(i*3,1) = -(2.0)*yb*cos(fi)*sin(theta)*cos(psi)-(2.0)*cos(fi)*cos(psi)*zb*cos(theta)+(2.0)*cos(fi)*sin(psi)*xb;
                dgdX(i*3,2) = -(2.0)*yb*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))-(2.0)*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*zb;
                dgdX(i*3,3) = -2.0;
                dgdX(i*3,4) = 0;
                dgdX(i*3,5) = 0;

                dgdX(i*3+1,0) = -(2.0)*cos(fi)*xb*cos(psi)-(2.0)*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb+(2.0)*yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta));
                dgdX(i*3+1,1) = -(2.0)*yb*sin(theta)*cos(psi)*sin(fi)-(2.0)*cos(psi)*sin(fi)*zb*cos(theta)+(2.0)*sin(psi)*xb*sin(fi);
                dgdX(i*3+1,2) = (2.0)*zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+(2.0)*yb*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta));
                dgdX(i*3+1,3) = 0;
                dgdX(i*3+1,4) = -2.0;
                dgdX(i*3+1,5) = 0;

                dgdX(i*3+2,0) = 0;
                dgdX(i*3+2,1) = (2.0)*yb*sin(psi)*sin(theta)+(2.0)*sin(psi)*zb*cos(theta)+(2.0)*xb*cos(psi);
                dgdX(i*3+2,2) = -(2.0)*yb*cos(psi)*cos(theta)+(2.0)*sin(theta)*cos(psi)*zb;
                dgdX(i*3+2,3) = 0;
                dgdX(i*3+2,4) = 0;
                dgdX(i*3+2,5) = -2.0;

                dgdX(setA.rows()*3+i*3,0) = -(2.0)*cos(psi)*sin(fi)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)+(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*cos(fi)*cos(psi)+(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta)))*cos(psi)*sin(fi)-(2.0)*cos(fi)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*cos(psi);
                dgdX(setA.rows()*3+i*3,1) = (2.0)*cos(fi)*cos(psi)*(yb*cos(fi)*sin(theta)*cos(psi)+cos(fi)*cos(psi)*zb*cos(theta)-cos(fi)*sin(psi)*xb)+(2.0)*sin(psi)*(yb*sin(psi)*sin(theta)+sin(psi)*zb*cos(theta)+xb*cos(psi))+(2.0)*cos(psi)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta)+za-z)+(2.0)*(yb*sin(theta)*cos(psi)*sin(fi)+cos(psi)*sin(fi)*zb*cos(theta)-sin(psi)*xb*sin(fi))*cos(psi)*sin(fi)+(2.0)*sin(psi)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*sin(fi)-(2.0)*cos(fi)*sin(psi)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x);
                dgdX(setA.rows()*3+i*3,2) = -(2.0)*sin(psi)*(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb)+(2.0)*cos(fi)*cos(psi)*(yb*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))+(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*zb)-(2.0)*cos(psi)*sin(fi)*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+yb*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta)));
                dgdX(setA.rows()*3+i*3,3) = (2.0)*cos(fi)*cos(psi);
                dgdX(setA.rows()*3+i*3,4) = (2.0)*cos(psi)*sin(fi);
                dgdX(setA.rows()*3+i*3,5) = -(2.0)*sin(psi);

                dgdX(setA.rows()*3+i*3+1,0) = -(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)-(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+(2.0)*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))+(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta)))*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi));
                dgdX(setA.rows()*3+i*3+1,1) = -(2.0)*(yb*sin(psi)*sin(theta)+sin(psi)*zb*cos(theta)+xb*cos(psi))*sin(theta)*cos(psi)+(2.0)*(yb*sin(theta)*cos(psi)*sin(fi)+cos(psi)*sin(fi)*zb*cos(theta)-sin(psi)*xb*sin(fi))*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+(2.0)*sin(psi)*sin(theta)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta)+za-z)-(2.0)*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*(yb*cos(fi)*sin(theta)*cos(psi)+cos(fi)*cos(psi)*zb*cos(theta)-cos(fi)*sin(psi)*xb)+(2.0)*cos(fi)*sin(theta)*cos(psi)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)-(2.0)*sin(theta)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*cos(psi)*sin(fi);
                dgdX(setA.rows()*3+i*3+1,2) = -(2.0)*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*(yb*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))+(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*zb)+(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))+(2.0)*sin(theta)*cos(psi)*(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb)-(2.0)*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+yb*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta)))*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))-(2.0)*cos(psi)*cos(theta)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta)+za-z)+(2.0)*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x);
                dgdX(setA.rows()*3+i*3+1,3) = -(2.0)*sin(fi)*cos(theta)+(2.0)*cos(fi)*sin(psi)*sin(theta);
                dgdX(setA.rows()*3+i*3+1,4) = (2.0)*cos(fi)*cos(theta)+(2.0)*sin(psi)*sin(theta)*sin(fi);
                dgdX(setA.rows()*3+i*3+1,5) = (2.0)*sin(theta)*cos(psi);

                dgdX(setA.rows()*3+i*3+2,0) = -(2.0)*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))-(2.0)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta)))*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))+(2.0)*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)+(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta));
                dgdX(setA.rows()*3+i*3+2,1) = (2.0)*cos(fi)*cos(psi)*cos(theta)*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)-(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*cos(psi)*sin(fi)*cos(theta)+(2.0)*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*(yb*cos(fi)*sin(theta)*cos(psi)+cos(fi)*cos(psi)*zb*cos(theta)-cos(fi)*sin(psi)*xb)-(2.0)*(yb*sin(theta)*cos(psi)*sin(fi)+cos(psi)*sin(fi)*zb*cos(theta)-sin(psi)*xb*sin(fi))*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))+(2.0)*sin(psi)*cos(theta)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta)+za-z)-(2.0)*(yb*sin(psi)*sin(theta)+sin(psi)*zb*cos(theta)+xb*cos(psi))*cos(psi)*cos(theta);
                dgdX(setA.rows()*3+i*3+2,2) = (2.0)*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*(cos(fi)*xb*cos(psi)+(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*zb-xa-yb*(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))+x)+(2.0)*(zb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi))+yb*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta)))*(cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))+(2.0)*sin(theta)*cos(psi)*(sin(psi)*xb-yb*sin(theta)*cos(psi)-cos(psi)*zb*cos(theta)+za-z)+(2.0)*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))*(yb*(sin(theta)*sin(fi)+cos(fi)*sin(psi)*cos(theta))+(sin(fi)*cos(theta)-cos(fi)*sin(psi)*sin(theta))*zb)+(2.0)*cos(psi)*(yb*cos(psi)*cos(theta)-sin(theta)*cos(psi)*zb)*cos(theta)+(2.0)*((cos(fi)*sin(theta)-sin(psi)*sin(fi)*cos(theta))*zb+ya-y-xb*cos(psi)*sin(fi)-yb*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi)))*(cos(fi)*cos(theta)+sin(psi)*sin(theta)*sin(fi));
                dgdX(setA.rows()*3+i*3+2,3) = (2.0)*sin(theta)*sin(fi)+(2.0)*cos(fi)*sin(psi)*cos(theta);
                dgdX(setA.rows()*3+i*3+2,4) = -(2.0)*cos(fi)*sin(theta)+(2.0)*sin(psi)*sin(fi)*cos(theta);
                dgdX(setA.rows()*3+i*3+2,5) = (2.0)*cos(psi)*cos(theta);
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

                dgdTheta(1,0) += 0.0;
                dgdTheta(1,1) += 2.0;
                dgdTheta(1,2) += 0.0;
                dgdTheta(1,3) += -(4.0)*qw*zb-(8.0)*yb*qx+(4.0)*qy*xb;
                dgdTheta(1,4) += (4.0)*qx*xb+(4.0)*qz*zb;
                dgdTheta(1,5) += (4.0)*qy*zb+(4.0)*qw*xb-(8.0)*yb*qz;

                dgdTheta(2,0) += 0.0;
                dgdTheta(2,1) += 0.0;
                dgdTheta(2,2) += 2.0;
                dgdTheta(2,3) += (4.0)*xb*qz-(8.0)*qx*zb+(4.0)*yb*qw;
                dgdTheta(2,4) += -(8.0)*qy*zb-(4.0)*qw*xb+(4.0)*yb*qz;
                dgdTheta(2,5) += (4.0)*qx*xb+(4.0)*qy*yb;

                dgdTheta(3,0) += (4.0)*qy*yb+(4.0)*qz*zb;
                dgdTheta(3,1) += -(4.0)*qw*zb-(8.0)*yb*qx+(4.0)*qy*xb;
                dgdTheta(3,2) += (4.0)*xb*qz-(8.0)*qx*zb+(4.0)*yb*qw;
                dgdTheta(3,3) += -(8.0)*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)*zb-(8.0)*yb*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))-(2.0)*(-(2.0)*xb*qz+(4.0)*qx*zb-(2.0)*yb*qw)*((2.0)*xb*qz-(4.0)*qx*zb+(2.0)*yb*qw)-(2.0)*((2.0)*qw*zb+(4.0)*yb*qx-(2.0)*qy*xb)*(-(2.0)*qw*zb-(4.0)*yb*qx+(2.0)*qy*xb)-(2.0)*((2.0)*qy*yb+(2.0)*qz*zb)*(-(2.0)*qy*yb-(2.0)*qz*zb);
                dgdTheta(3,4) += -(2.0)*((4.0)*qy*zb+(2.0)*qw*xb-(2.0)*yb*qz)*((2.0)*xb*qz-(4.0)*qx*zb+(2.0)*yb*qw)+(4.0)*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))*xb-(2.0)*((2.0)*qy*yb+(2.0)*qz*zb)*(-(2.0)*qw*zb-(2.0)*yb*qx+(4.0)*qy*xb)-(2.0)*(-(2.0)*qx*xb-(2.0)*qz*zb)*(-(2.0)*qw*zb-(4.0)*yb*qx+(2.0)*qy*xb)+(4.0)*yb*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x);
                dgdTheta(3,5) += -(2.0)*((2.0)*xb*qz-(4.0)*qx*zb+(2.0)*yb*qw)*(-(2.0)*qx*xb-(2.0)*qy*yb)-(2.0)*(-(2.0)*qy*zb-(2.0)*qw*xb+(4.0)*yb*qz)*(-(2.0)*qw*zb-(4.0)*yb*qx+(2.0)*qy*xb)+(4.0)*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)*xb-(2.0)*((4.0)*xb*qz-(2.0)*qx*zb+(2.0)*yb*qw)*((2.0)*qy*yb+(2.0)*qz*zb)+(4.0)*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x)*zb;

                dgdTheta(4,0) += (4.0)*qw*zb+(4.0)*yb*qx-(8.0)*qy*xb;
                dgdTheta(4,1) += (4.0)*qx*xb+(4.0)*qz*zb;
                dgdTheta(4,2) += -(8.0)*qy*zb-(4.0)*qw*xb+(4.0)*yb*qz;
                dgdTheta(4,3) += (4.0)*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))*xb-(2.0)*(-(2.0)*xb*qz+(4.0)*qx*zb-(2.0)*yb*qw)*(-(4.0)*qy*zb-(2.0)*qw*xb+(2.0)*yb*qz)-(2.0)*((2.0)*qw*zb+(2.0)*yb*qx-(4.0)*qy*xb)*(-(2.0)*qy*yb-(2.0)*qz*zb)+(4.0)*yb*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x)-(2.0)*((2.0)*qw*zb+(4.0)*yb*qx-(2.0)*qy*xb)*((2.0)*qx*xb+(2.0)*qz*zb);
                dgdTheta(4,4) += -(2.0)*(-(2.0)*qx*xb-(2.0)*qz*zb)*((2.0)*qx*xb+(2.0)*qz*zb)-(8.0)*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)*zb-(8.0)*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x)*xb-(2.0)*((2.0)*qw*zb+(2.0)*yb*qx-(4.0)*qy*xb)*(-(2.0)*qw*zb-(2.0)*yb*qx+(4.0)*qy*xb)-(2.0)*((4.0)*qy*zb+(2.0)*qw*xb-(2.0)*yb*qz)*(-(4.0)*qy*zb-(2.0)*qw*xb+(2.0)*yb*qz);
                dgdTheta(4,5) += -(2.0)*(-(2.0)*qy*zb-(2.0)*qw*xb+(4.0)*yb*qz)*((2.0)*qx*xb+(2.0)*qz*zb)-(2.0)*((4.0)*xb*qz-(2.0)*qx*zb+(2.0)*yb*qw)*((2.0)*qw*zb+(2.0)*yb*qx-(4.0)*qy*xb)+(4.0)*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))*zb+(4.0)*yb*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)-(2.0)*(-(4.0)*qy*zb-(2.0)*qw*xb+(2.0)*yb*qz)*(-(2.0)*qx*xb-(2.0)*qy*yb);

                dgdTheta(5,0) += -(8.0)*xb*qz+(4.0)*qx*zb-(4.0)*yb*qw;
                dgdTheta(5,1) += (4.0)*qy*zb+(4.0)*qw*xb-(8.0)*yb*qz;
                dgdTheta(5,2) += (4.0)*qx*xb+(4.0)*qy*yb;
                dgdTheta(5,3) += -(2.0)*(-(4.0)*xb*qz+(2.0)*qx*zb-(2.0)*yb*qw)*(-(2.0)*qy*yb-(2.0)*qz*zb)+(4.0)*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)*xb-(2.0)*((2.0)*qx*xb+(2.0)*qy*yb)*(-(2.0)*xb*qz+(4.0)*qx*zb-(2.0)*yb*qw)-(2.0)*((2.0)*qy*zb+(2.0)*qw*xb-(4.0)*yb*qz)*((2.0)*qw*zb+(4.0)*yb*qx-(2.0)*qy*xb)+(4.0)*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x)*zb;
                dgdTheta(5,4) += -(2.0)*((4.0)*qy*zb+(2.0)*qw*xb-(2.0)*yb*qz)*((2.0)*qx*xb+(2.0)*qy*yb)-(2.0)*((2.0)*qy*zb+(2.0)*qw*xb-(4.0)*yb*qz)*(-(2.0)*qx*xb-(2.0)*qz*zb)+(4.0)*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))*zb+(4.0)*yb*((1.0-(2.0)*pow(qx,2.0)-(2.0)*pow(qy,2.0))*zb-za+z+yb*((2.0)*qy*qz+(2.0)*qw*qx)+(-(2.0)*qy*qw+(2.0)*qx*qz)*xb)-(2.0)*(-(4.0)*xb*qz+(2.0)*qx*zb-(2.0)*yb*qw)*(-(2.0)*qw*zb-(2.0)*yb*qx+(4.0)*qy*xb);
                dgdTheta(5,5) += -(2.0)*((4.0)*xb*qz-(2.0)*qx*zb+(2.0)*yb*qw)*(-(4.0)*xb*qz+(2.0)*qx*zb-(2.0)*yb*qw)-(8.0)*yb*(xb*((2.0)*qw*qz+(2.0)*qy*qx)-ya+((2.0)*qy*qz-(2.0)*qw*qx)*zb+y+yb*(1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qx,2.0)))-(8.0)*((1.0-(2.0)*pow(qz,2.0)-(2.0)*pow(qy,2.0))*xb+yb*(-(2.0)*qw*qz+(2.0)*qy*qx)+((2.0)*qy*qw+(2.0)*qx*qz)*zb-xa+x)*xb-(2.0)*((2.0)*qx*xb+(2.0)*qy*yb)*(-(2.0)*qx*xb-(2.0)*qy*yb)-(2.0)*((2.0)*qy*zb+(2.0)*qw*xb-(4.0)*yb*qz)*(-(2.0)*qy*zb-(2.0)*qw*xb+(4.0)*yb*qz);

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
            Mat66 dgdThetaInv = dgdTheta.inverse();

            //std::cout << "Cx: \n" << Cx << "\n";
            //std::cout << "dgdX: \n" << dgdX << "\n";
            //std::cout << "dgdTheta: \n" << dgdTheta << "\n";
            uncertainty = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetaInv;
            //uncertainty = ConvertUncertaintyEuler2quat(uncertainty, transformation);
            return uncertainty;
        }

        /// convert [fi psi theta x y z] uncertainty matrix to [x y z qx qy qz]
        virtual const Mat66& ConvertUncertaintyEuler2quat(const Mat66& _uncertainty, const Mat34& transformation) {
            double fi = atan2(transformation.matrix()(1,0), transformation.matrix()(0,0));
            double psi = -asin(transformation.matrix()(2,0));
            double theta = atan2(transformation.matrix()(2,1), transformation.matrix()(2,2));
            float_type x = transformation.matrix()(0,3); float_type y = transformation.matrix()(1,3); float_type z = transformation.matrix()(2,3);

            Mat66 J;
            J(0,0) = 0; J(0,1) = 0; J(0,2) = 0; J(0,3) = 1; J(0,4) = 0; J(0,5) = 0;
            J(1,0) = 0; J(1,1) = 0; J(1,2) = 0; J(1,3) = 0; J(1,4) = 1; J(1,5) = 0;
            J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = 0; J(2,4) = 0; J(2,5) = 1;

            J(3,3) = 0; J(3,4) = 0; J(3,5) = 0;
            J(3,0) = -(0.5)*sin((0.5)*fi)*cos((0.5)*psi)*sin((0.5)*theta)-(0.5)*sin((0.5)*psi)*cos((0.5)*theta)*cos((0.5)*fi);
            J(3,1) = -(0.5)*cos((0.5)*theta)*sin((0.5)*fi)*cos((0.5)*psi)-(0.5)*sin((0.5)*psi)*cos((0.5)*fi)*sin((0.5)*theta);
            J(3,2) = (0.5)*sin((0.5)*psi)*sin((0.5)*fi)*sin((0.5)*theta)+(0.5)*cos((0.5)*theta)*cos((0.5)*psi)*cos((0.5)*fi);

            J(4,0) = (0.5)*cos((0.5)*psi)*cos((0.5)*fi)*sin((0.5)*theta)-(0.5)*sin((0.5)*psi)*cos((0.5)*theta)*sin((0.5)*fi);
            J(4,1) = -(0.5)*sin((0.5)*psi)*sin((0.5)*fi)*sin((0.5)*theta)+(0.5)*cos((0.5)*theta)*cos((0.5)*psi)*cos((0.5)*fi);
            J(4,2) = (0.5)*cos((0.5)*theta)*sin((0.5)*fi)*cos((0.5)*psi)-(0.5)*sin((0.5)*psi)*cos((0.5)*fi)*sin((0.5)*theta);
            J(4,3) = 0; J(4,3) = 0; J(4,4) = 0;

            J(5,3) = 0; J(5,4) = 0; J(5,5) = 0;
            J(5,0) = (0.5)*sin((0.5)*psi)*sin((0.5)*fi)*sin((0.5)*theta)+(0.5)*cos((0.5)*theta)*cos((0.5)*psi)*cos((0.5)*fi);
            J(5,1) = -(0.5)*cos((0.5)*psi)*cos((0.5)*fi)*sin((0.5)*theta)-(0.5)*sin((0.5)*psi)*cos((0.5)*theta)*sin((0.5)*fi);
            J(5,2) = -(0.5)*sin((0.5)*fi)*cos((0.5)*psi)*sin((0.5)*theta)-(0.5)*sin((0.5)*psi)*cos((0.5)*theta)*cos((0.5)*fi);

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

