/** @file grabber.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "../Defs/putslam_defs.h"
#include <iostream>
#include <string>
#include <vector>

namespace putslam {
	/// Grabber interface
	class Grabber {
        public:

            /// Grabber type
            enum Type {
                    /// RGB camera
                    TYPE_RGB,
                    /// 2D Depth sensor
                    TYPE_2D_DEPTH,
                    /// 3D Depth sensor
                    TYPE_3D_DEPTH,
                    /// PrimeSense-based (Kinect, Asus, PrimeSense)
                    TYPE_PRIMESENSE
            };

            /// overloaded constructor
            Grabber(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the grabber
            virtual const std::string& getName() const = 0;

            /// Returns the current point cloud
            virtual const Point3D::Cloud& getCloud(void) const = 0;

            /// Returns the current 2D image
            virtual const Image& getImage(void) const = 0;

            /// Grab image and/or point cloud
            virtual void grab() = 0;

            /// run grabber thread
            virtual void run() = 0;

            /// Virtual descrutor
            virtual ~Grabber() {}

        protected:
            /// Grabber type
            Type type;

            /// Grabber name
            const std::string name;

            /// RGBZXYZ Point cloud
            Point3D::Cloud cloud;

            /// 2D image
            Image image;
	};
};

#endif // _GRABBER_H_
