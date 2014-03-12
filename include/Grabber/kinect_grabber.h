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

        void getPoint(uint_fast16_t u, uint_fast16_t v, uint_fast16_t depth, Eigen::Vector3d& point3D){
            Eigen::Vector3d point(u, v, depth);
            point3D = PHCPModel*point;
        }

        void computeCov(uint_fast16_t u, uint_fast16_t v, uint_fast16_t depth, Mat33& cov){
            //float_type dispDer = config.k3 * 1/(config.k2*pow(cos((disparity/config.k2) + config.k1),2.0));
            Mat33 J;
            J << 0.0017*depth, 0, (0.0017*u-0.549),
                 0, 0.0017*depth, (0.0017*v-0.443),
                 0, 0, 1;
            Ruvd(2,2) = config.distVarCoefs[0]*pow(depth,3.0) + config.distVarCoefs[1]*pow(depth,2.0) + config.distVarCoefs[2]*depth + config.distVarCoefs[3];
            cov=J*Ruvd*J.transpose();
        }

        class Config{
          public:
            Config() :
                focalLength{582.64, 586.97},
                focalAxis{320.17, 260.0},
                varU(1.1046), varV(0.64160),
                distVarCoefs{-8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3}{
            }
            public:
                float_type focalLength[2];
                float_type focalAxis[2];
                float_type varU, varV;// variance u,v
                float_type distVarCoefs[4];
        };

        UncertaintyModel(){
            PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],
                          0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1],
                          0,0,1;
            Ruvd << config.varU, 0, 0,
                    0, config.varV, 0,
                    0, 0, 0;
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
