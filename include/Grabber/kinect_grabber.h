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
    public:
        /// Pointer
        typedef std::unique_ptr<KinectGrabber> Ptr;

        /// Construction
        KinectGrabber(void);

        /// Name of the grabber
        virtual const std::string& getName() const;

        /// Returns current point cloud
        virtual const Point3D::Cloud& getCloud(void) const;

        /// Returns the current 2D image
        virtual const SensorFrame& getSensorFrame(void) const;

        /// Grab image and/or point cloud
        virtual void grab();

        /// Calibrate sensor
        virtual void calibrate(void);

    private:
};

#endif // KINECT_GRABBER_H_INCLUDED
