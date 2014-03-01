/** @file kinect_grabber.h
 *
 * implementation - Kinect Grabber
 *
 */

#ifndef KINECT_GRABBER_H_INCLUDED
#define KINECT_GRABBER_H_INCLUDED

#include "grabber.h"
#include <iostream>
#include <memory>

namespace putslam {
	/// create a single grabber (Kinect)
	Grabber* createGrabberKinect(void);
};

using namespace putslam;

/// Grabber implementation
class KinectGrabber : public Grabber {
    class UncertaintyModel {
      public:
        inline float_type computeDepth(uint_fast16_t disparity){
            return config.k3 * tan((disparity/config.k2) + config.k1);
        }

        void getPoint(uint_fast16_t u, uint_fast16_t v, uint_fast16_t disparity, Eigen::Vector3d& point3D){
            Eigen::Vector3d point(u,v,disparity);
            point3D = PHCPModel*point;
        }

        void computeCov(uint_fast16_t u, uint_fast16_t v, uint_fast16_t disparity, Mat33& cov){
            float_type dispDer = config.k3 * 1/(config.k2*pow(cos((disparity/config.k2) + config.k1),2.0));
            Mat33 J;
            J << 0.0017*computeDepth(disparity), 0, dispDer*(0.0017*u-0.549),
                 0, 0.0017*computeDepth(disparity), dispDer*(0.0017*v-0.443),
                 0, 0, dispDer;
            cov=J*Ruvd*J.transpose();
        }

        class Config{
          public:
            Config() : k1(1.1863),
                k2(2842.5),
                k3(0.1236),
                focalLength{582.64, 586.97},
                focalAxis{320.17, 260.0},
                varU(1.1046), varV(0.64160), varD(1.6028){
            }
            public:
                float_type k1, k2, k3; // depth = k3 * tan(disparity/k2 + k1);
                float_type focalLength[2];
                float_type focalAxis[2];
                float_type varU, varV, varD;// variance u,v,disparity
        };

        UncertaintyModel(){
            PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],
                          0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1],
                          0,0,1;
            Ruvd << config.varU, 0, 0,
                    0, config.varV, 0,
                    0, 0, config.varD;
        }

        private:
            Config config;
            Mat33 PHCPModel;//pin-hole camera projection model
            Mat33 Ruvd; //covariance matrix for [u,v,disp]
    };

    public:
        /// Pointer
        typedef std::unique_ptr<KinectGrabber> Ptr;

        /// Construction
        KinectGrabber(void);

        /// Name of the grabber
        virtual const std::string& getName() const;

        /// Returns current point cloud
        virtual const PointCloud& getCloud(void) const;

        /// Returns the current 2D image
        virtual const SensorFrame& getSensorFrame(void) const;

        /// Grab image and/or point cloud
        virtual void grab();

        /// Calibrate sensor
        virtual void calibrate(void);

        UncertaintyModel model;

    private:
};

#endif // KINECT_GRABBER_H_INCLUDED
