/** @file grabber.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "../Defs/putslam_defs.h"
#include "calibration.h"
#include "depthSensorModel.h"
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
            Grabber(const std::string _name, Type _type, Mode _mode) :  type(_type), mode(_mode), name(_name) {};

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

            /// Create point cloud from current RGB and depth image
            virtual void convert2cloud(const DepthSensorModel& model, PointCloud& cloud) {
                cloud.clear();
                if (type==TYPE_PRIMESENSE){
                    for (int i=0;i<sensorFrame.rgbImage.rows;i++){
                        for (int j=0;j<sensorFrame.rgbImage.cols;j++){
                            cv::Point3_ <uchar>* p = sensorFrame.rgbImage.ptr<cv::Point3_<uchar> >(i,j);
                            putslam::Point3D point;
                            Eigen::Vector3d pointxyz;
                            model.getPoint(j,i, (double)sensorFrame.depthImage.at<uint16_t>(i, j)/5000.0, pointxyz);//5000 - depth scale
                            point.x = pointxyz(0); point.y = pointxyz(1); point.z = pointxyz(2);
                            point.r = p->z; point.g = p->y; point.b = p->x;
                            cloud.push_back(point);
                            //getchar();
                        }
                    }
                }
                else{
                    std::cout << "convert to cloud: sensor type not supported.\n";
                }
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
