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
#include <queue>
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

            /// Mode
            enum Mode {
                    /// get only last frame
                    MODE_CONTINUOUS,
                    /// buffer all frames in memory
                    MODE_BUFFER,
            };

            /// overloaded constructor
            Grabber(const std::string _name, Type _type, Mode _mode) : name(_name), type(_type), mode(_mode) {};

            /// Name of the grabber
            virtual const std::string& getName() const = 0;

            /// Returns the current point cloud
            virtual const PointCloud& getCloud(void) const = 0;

            /// Returns the current 2D image
            virtual const SensorFrame& getSensorFrame(void) {
                if (mode==MODE_BUFFER){
                    mtx.lock();
                    sensorFrame = sensorFrames.front();
                    sensorFrames.pop();
                    mtx.unlock();
                }
                return sensorFrame;
            }

            /// Grab image and/or point cloud
            virtual bool grab() = 0;

            /// Calibrate sensor
            virtual void calibrate() = 0;

            /// Closing a device
            virtual int grabberClose() = 0;

            /// Return starting position of sensor
            virtual Eigen::Matrix4f getStartingSensorPose() = 0;

            /// Virtual destructor
            virtual ~Grabber() {}

        protected:
            /// Grabber type
            Type type;

            /// Operation mode
            Mode mode;

            /// Grabber name
            const std::string name;

            /// RGBZXYZ Point cloud
            PointCloud cloud;

            /// Sensor frame
            SensorFrame sensorFrame;

            /// sequence
            std::queue<SensorFrame> sensorFrames;

            /// Calibration methods
            Calibration calibrator;

            /// Mutex
            std::mutex mtx;
	};
};

#endif // _GRABBER_H_
