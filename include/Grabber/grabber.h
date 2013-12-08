/** @file grabber.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace handest {
	/// Grabber interface
	class Grabber {
	public:

		/// Name of the grabber
		virtual const std::string& getName() const = 0;

		/// Returns the current point cloud
		virtual void getCloud(Point3D::Cloud& current_cloud) const = 0;

		/// Grab point cloud
		virtual void grab() = 0;

		/// Virtual descrutor
		virtual ~Grabber() {}
	};
};

#endif // _GRABBER_H_
