/** @file grabber.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "../Defs/putslam_defs.h"
#include "calibration.h"
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

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
                    TYPE_PRIMESENSE,
                    /// Read data from files
                    TYPE_FILE
            };

            /// overloaded constructor
            Grabber(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the grabber
            virtual const std::string& getName() const = 0;

            /// Returns the current point cloud
            virtual const PointCloud& getCloud(void) const = 0;

            /// Returns the current 2D image
            virtual const SensorFrame& getSensorFrame(void) const = 0;

            /// Grab image and/or point cloud
            virtual void grab() = 0;

            /// Calibrate sensor
            virtual void calibrate() = 0;

            ///Clossing a device
            virtual int grabberClose() = 0;

            /// Virtual descrutor
            virtual ~Grabber() {}

        protected:
            /// Grabber type
            Type type;

            /// Grabber name
            const std::string name;

            /// RGBZXYZ Point cloud
            PointCloud cloud;

            /// 2D image
            SensorFrame sensor_frame;

            /// Calibration methods
            Calibration calibrator;

            /// Mutex
            std::mutex mtx;
	};
};

#endif // _GRABBER_H_
