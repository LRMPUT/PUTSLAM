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

            /// Grab image and/or point cloud
            virtual const Mat34& computeTransformation(void) = 0;

            /// Virtual descrutor
            virtual ~TransformEst() {}

        protected:
            /// Compute uncertainty matrix [6x6] (x,y,z,qx,qy,qz)
            virtual const Mat66& computeUncertainty(std::vector<Point3D> setA, std::vector<Mat33> setAUncertainty, std::vector<Point3D> setB, std::vector<Mat33> setBUncertainty, Mat34 transformation) {
                Mat66 dgdTheta;
                Eigen::Matrix<double,3,1> euler = transformation.rotation().eulerAngles(2, 1, 0);//z-y-x convention
                double fi = euler(2); double psi = euler(1); double theta = euler(0);
                float_type x = transformation.matrix()(0,3); float_type y = transformation.matrix()(1,3); float_type z = transformation.matrix()(2,3);
                Eigen::MatrixXd Cx(2*3*setA.size(),2*3*setA.size());
                Cx = Eigen::ArrayXXd::Zero(2*3*setA.size(), 2*3*setA.size());
                Eigen::MatrixXd dgdX(2*3*setA.size(),6);
                for (size_t i=0;i<setA.size();i++){
                    float_type x = transformation(0,3); float_type y = transformation(1,3);  float_type z = transformation(2,3);
                    float_type xa = setA[i].x; float_type ya = setA[i].y; float_type za = setA[i].z;
                    float_type xb = setB[i].x; float_type yb = setB[i].y; float_type zb = setB[i].z;
                    float_type sinfi = sin(fi); float_type cosfi = cos(fi);
                    float_type sinpsi = sinpsi; float_type cospsi = cos(psi);
                    float_type sintheta = sin(theta); float_type costheta = cos(theta);

                    dgdTheta(0,0) = (2.0)*pow((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)),2.0)+(2.0)*pow(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)),2.0)-(2.0)*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb)+(2.0)*pow(yb*cos(psi)*cos(fi)-sin(fi)*cos(psi)*zb,2.0)-(2.0)*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi)-za-z-xb*sin(psi))*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi))+(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb);
                    dgdTheta(0,1) = -(2.0)*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi)-za-z-xb*sin(psi))*(yb*cos(fi)*sin(psi)-sin(fi)*zb*sin(psi))-(2.0)*(yb*cos(psi)*cos(fi)-sin(fi)*cos(psi)*zb)*(zb*cos(fi)*sin(psi)+cos(psi)*xb+yb*sin(fi)*sin(psi))+(2.0)*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(sin(fi)*cos(psi)*sin(theta)*zb-yb*cos(psi)*sin(theta)*cos(fi))-(2.0)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))-(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))+(2.0)*(cos(theta)*sin(fi)*cos(psi)*zb-yb*cos(theta)*cos(psi)*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x);
                    dgdTheta(0,2) = -(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))+(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb)-(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)-(2.0)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb);
                    dgdTheta(0,3) = -(2.0)*yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))-(2.0)*zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi));
                    dgdTheta(0,4) = (2.0)*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-(2.0)*yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi));
                    dgdTheta(0,5) = -(2.0)*yb*cos(psi)*cos(fi)+(2.0)*sin(fi)*cos(psi)*zb;

                    dgdTheta(1,0) = -(2.0)*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi)-za-z-xb*sin(psi))*(yb*cos(fi)*sin(psi)-sin(fi)*zb*sin(psi))-(2.0)*(yb*cos(psi)*cos(fi)-sin(fi)*cos(psi)*zb)*(zb*cos(fi)*sin(psi)+cos(psi)*xb+yb*sin(fi)*sin(psi))+(2.0)*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(sin(fi)*cos(psi)*sin(theta)*zb-yb*cos(psi)*sin(theta)*cos(fi))-(2.0)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))-(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))+(2.0)*(cos(theta)*sin(fi)*cos(psi)*zb-yb*cos(theta)*cos(psi)*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x);
                    dgdTheta(1,1) = (2.0)*pow(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi),2.0)-(2.0)*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi)-za-z-xb*sin(psi))*(cos(psi)*zb*cos(fi)+yb*sin(fi)*cos(psi)-xb*sin(psi))+(2.0)*(sin(theta)*zb*cos(fi)*sin(psi)+yb*sin(fi)*sin(theta)*sin(psi)+cos(psi)*xb*sin(theta))*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)+(2.0)*pow(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi),2.0)+(2.0)*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)*(yb*cos(theta)*sin(fi)*sin(psi)+cos(theta)*zb*cos(fi)*sin(psi)+cos(theta)*cos(psi)*xb)+(2.0)*pow(zb*cos(fi)*sin(psi)+cos(psi)*xb+yb*sin(fi)*sin(psi),2.0);
                    dgdTheta(1,2) = (2.0)*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)+(2.0)*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)+(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))-(2.0)*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb);
                    dgdTheta(1,3) = (2.0)*cos(theta)*xb*sin(psi)-(2.0)*yb*cos(theta)*sin(fi)*cos(psi)-(2.0)*cos(theta)*cos(psi)*zb*cos(fi);
                    dgdTheta(1,4) = -(2.0)*yb*sin(fi)*cos(psi)*sin(theta)+(2.0)*xb*sin(theta)*sin(psi)-(2.0)*cos(psi)*sin(theta)*zb*cos(fi);
                    dgdTheta(1,5) = (2.0)*zb*cos(fi)*sin(psi)+(2.0)*cos(psi)*xb+(2.0)*yb*sin(fi)*sin(psi);

                    dgdTheta(2,0) = -(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))+(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb)-(2.0)*((sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi)))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)-(2.0)*(yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))+zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi)))*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb);
                    dgdTheta(2,1) = (2.0)*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)+(2.0)*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)+(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(cos(theta)*xb*sin(psi)-yb*cos(theta)*sin(fi)*cos(psi)-cos(theta)*cos(psi)*zb*cos(fi))-(2.0)*(yb*sin(fi)*cos(psi)*sin(theta)-xb*sin(theta)*sin(psi)+cos(psi)*sin(theta)*zb*cos(fi))*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb);
                    dgdTheta(2,2) = (2.0)*pow(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb,2.0)-(2.0)*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb)*(yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))+xa-(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-cos(theta)*cos(psi)*xb+x)+(2.0)*pow(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb,2.0)+(2.0)*(yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+cos(psi)*xb*sin(theta)+(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb)*(ya+y-yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))-cos(psi)*xb*sin(theta)-(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb);
                    dgdTheta(2,3) = (2.0)*yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+(2.0)*cos(psi)*xb*sin(theta)+(2.0)*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb;
                    dgdTheta(2,4) = (2.0)*yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(2.0)*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-(2.0)*cos(theta)*cos(psi)*xb;
                    dgdTheta(2,5) = 0;

                    dgdTheta(3,0) = -(2.0)*yb*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))-(2.0)*zb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi));
                    dgdTheta(3,1) = (2.0)*cos(theta)*xb*sin(psi)-(2.0)*yb*cos(theta)*sin(fi)*cos(psi)-(2.0)*cos(theta)*cos(psi)*zb*cos(fi);
                    dgdTheta(3,2) = (2.0)*yb*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))+(2.0)*cos(psi)*xb*sin(theta)+(2.0)*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi))*zb;
                    dgdTheta(3,3) = 2.0;
                    dgdTheta(3,4) = 0;
                    dgdTheta(3,5) = 0;

                    dgdTheta(4,0) = (2.0)*(sin(fi)*sin(theta)*sin(psi)+cos(theta)*cos(fi))*zb-(2.0)*yb*(sin(theta)*cos(fi)*sin(psi)-cos(theta)*sin(fi));
                    dgdTheta(4,1) = -(2.0)*yb*sin(fi)*cos(psi)*sin(theta)+(2.0)*xb*sin(theta)*sin(psi)-(2.0)*cos(psi)*sin(theta)*zb*cos(fi);
                    dgdTheta(4,2) = (2.0)*yb*(sin(theta)*cos(fi)-cos(theta)*sin(fi)*sin(psi))-(2.0)*(cos(theta)*cos(fi)*sin(psi)+sin(fi)*sin(theta))*zb-(2.0)*cos(theta)*cos(psi)*xb;
                    dgdTheta(4,3) = 0;
                    dgdTheta(4,4) = 2.0;
                    dgdTheta(4,5) = 0;

                    dgdTheta(5,0) = -(2.0)*yb*cos(psi)*cos(fi)+(2.0)*sin(fi)*cos(psi)*zb;
                    dgdTheta(5,1) = (2.0)*zb*cos(fi)*sin(psi)+(2.0)*cos(psi)*xb+(2.0)*yb*sin(fi)*sin(psi);
                    dgdTheta(5,2) = 0;
                    dgdTheta(5,3) = 0;
                    dgdTheta(5,4) = 0;
                    dgdTheta(5,5) = 2.0;
                    Cx.block<3,3>(i*3,i*3) = setAUncertainty[i];
                    Cx.block<3,3>(setA.size()*3+i*3,setA.size()*3+i*3) = setBUncertainty[i];

                    dgdX(i*3,0) = -(2.0)*yb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-(2.0)*zb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi));
                    dgdX(i*3,1) = -(2.0)*cos(fi)*zb*cos(theta)*cos(psi)-(2.0)*cos(theta)*yb*sin(fi)*cos(psi)+(2.0)*sin(psi)*cos(theta)*xb;
                    dgdX(i*3,2) = (2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb+(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb+(2.0)*xb*cos(psi)*sin(theta);
                    dgdX(i*3,3) = 2.0;
                    dgdX(i*3,4) = 0;
                    dgdX(i*3,5) = 0;

                    dgdX(i*3+1,0) = -(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*yb+(2.0)*zb*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta));
                    dgdX(i*3+1,1) = -(2.0)*cos(fi)*zb*cos(psi)*sin(theta)-(2.0)*yb*sin(fi)*cos(psi)*sin(theta)+(2.0)*sin(psi)*xb*sin(theta);
                    dgdX(i*3+1,2) = (2.0)*yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-(2.0)*zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-(2.0)*cos(theta)*xb*cos(psi);
                    dgdX(i*3+1,3) = 0;
                    dgdX(i*3+1,4) = 2.0;
                    dgdX(i*3+1,5) = 0;

                    dgdX(i*3+2,0) = (2.0)*zb*sin(fi)*cos(psi)-(2.0)*cos(fi)*yb*cos(psi);
                    dgdX(i*3+2,1) = (2.0)*sin(psi)*yb*sin(fi)+(2.0)*cos(fi)*zb*sin(psi)+(2.0)*xb*cos(psi);
                    dgdX(i*3+2,2) = 0;
                    dgdX(i*3+2,3) = 0;
                    dgdX(i*3+2,4) = 0;
                    dgdX(i*3+2,5) = 2.0;

                    dgdX(setA.size()+i*3+2,0) = (2.0)*cos(theta)*(yb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))+zb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi)))*cos(psi)+(2.0)*(zb*sin(fi)*cos(psi)-cos(fi)*yb*cos(psi))*sin(psi)+(2.0)*cos(psi)*((cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*yb-zb*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta)))*sin(theta);
                    dgdX(setA.size()+i*3+2,1) = (2.0)*(sin(psi)*xb-yb*sin(fi)*cos(psi)-cos(fi)*zb*cos(psi)+za+z)*cos(psi)+(2.0)*cos(theta)*(cos(fi)*zb*cos(theta)*cos(psi)+cos(theta)*yb*sin(fi)*cos(psi)-sin(psi)*cos(theta)*xb)*cos(psi)+(2.0)*(cos(fi)*zb*cos(psi)*sin(theta)+yb*sin(fi)*cos(psi)*sin(theta)-sin(psi)*xb*sin(theta))*cos(psi)*sin(theta)+(2.0)*sin(psi)*cos(theta)*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))+(2.0)*(sin(psi)*yb*sin(fi)+cos(fi)*zb*sin(psi)+xb*cos(psi))*sin(psi)-(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*sin(psi)*sin(theta);
                    dgdX(setA.size()+i*3+2,2) = -(2.0)*(yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))*cos(psi)*sin(theta)+(2.0)*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))*cos(psi)*sin(theta)-(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb+xb*cos(psi)*sin(theta))*cos(theta)*cos(psi)+(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*cos(theta)*cos(psi);
                    dgdX(setA.size()+i*3+2,3) = -(2.0)*cos(theta)*cos(psi);
                    dgdX(setA.size()+i*3+2,4) = -(2.0)*cos(psi)*sin(theta);
                    dgdX(setA.size()+i*3+2,5) = (2.0)*sin(psi);

                    dgdX(setA.size()+i*3+1,0) = -(2.0)*(zb*sin(fi)*cos(psi)-cos(fi)*yb*cos(psi))*sin(fi)*cos(psi)+(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*((cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*yb-zb*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta)))-(2.0)*cos(fi)*(sin(psi)*xb-yb*sin(fi)*cos(psi)-cos(fi)*zb*cos(psi)+za+z)*cos(psi)-(2.0)*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))-(2.0)*(yb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))+zb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi)))*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))+(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta));
                    dgdX(setA.size()+i*3+1,1) = (2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*sin(fi)*cos(psi)*sin(theta)-(2.0)*(sin(psi)*yb*sin(fi)+cos(fi)*zb*sin(psi)+xb*cos(psi))*sin(fi)*cos(psi)-(2.0)*cos(theta)*sin(fi)*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))*cos(psi)+(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*(cos(fi)*zb*cos(psi)*sin(theta)+yb*sin(fi)*cos(psi)*sin(theta)-sin(psi)*xb*sin(theta))-(2.0)*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))*(cos(fi)*zb*cos(theta)*cos(psi)+cos(theta)*yb*sin(fi)*cos(psi)-sin(psi)*cos(theta)*xb)+(2.0)*sin(psi)*(sin(psi)*xb-yb*sin(fi)*cos(psi)-cos(fi)*zb*cos(psi)+za+z)*sin(fi);
                    dgdX(setA.size()+i*3+1,2) = -(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*(yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))+(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb+xb*cos(psi)*sin(theta))*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))+(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi));
                    dgdX(setA.size()+i*3+1,3) = (2.0)*cos(fi)*sin(theta)-(2.0)*sin(psi)*cos(theta)*sin(fi);
                    dgdX(setA.size()+i*3+1,4) = -(2.0)*cos(fi)*cos(theta)-(2.0)*sin(psi)*sin(fi)*sin(theta);
                    dgdX(setA.size()+i*3+1,5) = -(2.0)*sin(fi)*cos(psi);

                    dgdX(setA.size()+i*3+2,0) = (2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*((cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*yb-zb*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta)))-(2.0)*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))-(2.0)*cos(fi)*(zb*sin(fi)*cos(psi)-cos(fi)*yb*cos(psi))*cos(psi)+(2.0)*(yb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))+zb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi)))*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))+(2.0)*(sin(psi)*xb-yb*sin(fi)*cos(psi)-cos(fi)*zb*cos(psi)+za+z)*sin(fi)*cos(psi)-(2.0)*(cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta));
                    dgdX(setA.size()+i*3+2,1) = -(2.0)*cos(fi)*cos(theta)*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))*cos(psi)+(2.0)*cos(fi)*sin(psi)*(sin(psi)*xb-yb*sin(fi)*cos(psi)-cos(fi)*zb*cos(psi)+za+z)+(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*(cos(fi)*zb*cos(psi)*sin(theta)+yb*sin(fi)*cos(psi)*sin(theta)-sin(psi)*xb*sin(theta))+(2.0)*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))*(cos(fi)*zb*cos(theta)*cos(psi)+cos(theta)*yb*sin(fi)*cos(psi)-sin(psi)*cos(theta)*xb)-(2.0)*cos(fi)*(sin(psi)*yb*sin(fi)+cos(fi)*zb*sin(psi)+xb*cos(psi))*cos(psi)+(2.0)*cos(fi)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*cos(psi)*sin(theta);
                    dgdX(setA.size()+i*3+2,2) = (2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb-ya+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb-y+xb*cos(psi)*sin(theta))*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*(yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi))-(2.0)*((cos(fi)*cos(theta)+sin(psi)*sin(fi)*sin(theta))*yb+(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*zb+xb*cos(psi)*sin(theta))*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))+(2.0)*(cos(fi)*sin(psi)*sin(theta)-cos(theta)*sin(fi))*(xa+x+yb*(cos(fi)*sin(theta)-sin(psi)*cos(theta)*sin(fi))-zb*(cos(fi)*sin(psi)*cos(theta)+sin(fi)*sin(theta))-cos(theta)*xb*cos(psi));
                    dgdX(setA.size()+i*3+2,3) = -(2.0)*cos(fi)*sin(psi)*cos(theta)-(2.0)*sin(fi)*sin(theta);
                    dgdX(setA.size()+i*3+2,4) = -(2.0)*cos(fi)*sin(psi)*sin(theta)+(2.0)*cos(theta)*sin(fi);
                    dgdX(setA.size()+i*3+2,5) = -(2.0)*cos(fi)*cos(psi);
                }
                Mat66 dgdThetaInv = dgdTheta.inverse();
                uncertainty = dgdThetaInv*dgdX.transpose()*Cx*dgdX*dgdThetaInv;
                return uncertainty;
            }

            /// Estimated transformation
            Mat34 transformation;
            ///Uncertainty
            Mat66 uncertainty;
    };
};

#endif // _TRANSFORMEST_H_

