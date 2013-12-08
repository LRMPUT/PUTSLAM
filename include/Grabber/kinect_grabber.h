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

namespace handest {
	/// create a single grabber (Kinect)
	Grabber* createGrabberKinect(void);
};

using namespace handest;

/// Grabber implementation
class KinectGrabber : public Grabber {
    public:
	/// Pointer
	typedef std::unique_ptr<KinectGrabber> Ptr;

	/// Construction
	KinectGrabber(void);

	/// Name of the grabber
	virtual const std::string& getName() const;

	/// Returns the current point cloud
	virtual void getCloud(Point3D::Cloud& current_cloud) const;

	/// Grab point cloud
	virtual void grab();

    protected:
	/// RGBZXYZ Point cloud
	Point3D::Cloud cloud;
	/// Grabber name
	const std::string name;
};

#endif // KINECT_GRABBER_H_INCLUDED
