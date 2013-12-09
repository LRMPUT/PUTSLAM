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

namespace putslam {
	/// Grabber interface
	class Grabber {
        public:

            /// Grabber type
            enum Type {
                    /// RGB camera */
                    TYPE_RGB,
                    /// 2D Depth sensor
                    TYPE_2D_DEPTH,
                    /// 3D Depth sensor
                    TYPE_3D_DEPTH,
                    /// PrimeSense-based (Kinect, Asus, PrimeSense)
                    TYPE_PRIMESENSE
            };

            /// Name of the grabber
            virtual const std::string& getName() const = 0;

            /// Returns the current point cloud
            virtual void getCloud(Point3D::Cloud& current_cloud) const = 0;

            /// Returns the current point cloud
            virtual void getImage(Image& current_image) const = 0;

            /// Grab point cloud
            virtual void grab() = 0;

            /// run grabber thread
            virtual void run() = 0;

            /// Virtual descrutor
            virtual ~Grabber() {}
	};
};

#endif // _GRABBER_H_
