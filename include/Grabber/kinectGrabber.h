/** @file kinect_grabber.h
 *
 * implementation - Kinect Grabber
 *
 */

#ifndef KINECT_GRABBER_H_INCLUDED
#define KINECT_GRABBER_H_INCLUDED

#include "grabber.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "depthSensorModel.h"
#include <iostream>
#include <memory>

namespace putslam {
	/// create a single grabber (Kinect)
	Grabber* createGrabberKinect(void);
    Grabber* createGrabberKinect(std::string configFile, Grabber::Mode mode);
};

using namespace putslam;

/// Grabber implementation
class KinectGrabber : public Grabber {
    public:

        /// Pointer
        typedef std::unique_ptr<KinectGrabber> Ptr;

        /// Construction
        KinectGrabber(void);

        /// Construction
        KinectGrabber(std::string modelFilename, Mode _model) : Grabber("Kinect Grabber", TYPE_PRIMESENSE, _model), model(modelFilename){
        }

        /// Name of the grabber
        virtual const std::string& getName() const;

        /// Returns current point cloud
        virtual const PointCloud& getCloud(void) const;

        /// Grab image and/or point cloud
        virtual bool grab();

        /// Calibrate sensor
        virtual void calibrate(void);

        ///Sensor uninitialize
        virtual int grabberClose(void);

        /// Return starting position of sensor TODO: Implement
        Eigen::Matrix4f getStartingSensorPose(){};

    private:
        /// Sensor model
        DepthSensorModel model;

};

#endif // KINECT_GRABBER_H_INCLUDED
