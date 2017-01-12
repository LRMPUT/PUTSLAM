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
#ifdef BUILD_KINECT
#include <libfreenect/libfreenect.hpp>
#endif
namespace putslam {
	/// create a single grabber (Kinect)
	Grabber* createGrabberKinect(void);
    Grabber* createGrabberKinect(std::string configFile, Grabber::Mode mode);
};

using namespace putslam;

#ifdef BUILD_KINECT

class myMutex {
    public:
        myMutex() {
            pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
            pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
            pthread_mutex_unlock( &m_mutex );
        }
    private:
        pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
    public:
        MyFreenectDevice(freenect_context *_ctx, int _index)
            : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
            m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
            m_new_depth_frame(false), depthMat(cv::Size(640,480),CV_16UC1),
            rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
            ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)) {

            for( unsigned int i = 0 ; i < 2048 ; i++) {
                float v = i/2048.0;
                v = std::pow(v, 3)* 6;
                m_gamma[i] = v*6*256;
            }
        }

        // Do not call directly even in child
        void VideoCallback(void* _rgb, uint32_t timestamp) {
            std::cout << "RGB callback" << std::endl;
            m_rgb_mutex.lock();
            uint8_t* rgb = static_cast<uint8_t*>(_rgb);
            rgbMat.data = rgb;
            m_new_rgb_frame = true;
            m_rgb_mutex.unlock();
        };

        // Do not call directly even in child
        void DepthCallback(void* _depth, uint32_t timestamp) {
            std::cout << "Depth callback" << std::endl;
            m_depth_mutex.lock();
            uint16_t* depth = static_cast<uint16_t*>(_depth);
            depthMat.data = (uchar*) depth;
            m_new_depth_frame = true;
            m_depth_mutex.unlock();
        }

        bool getVideo(cv::Mat& output) {
            m_rgb_mutex.lock();
            if(m_new_rgb_frame) {
                cv::cvtColor(rgbMat, output, CV_RGB2BGR);
                m_new_rgb_frame = false;
                m_rgb_mutex.unlock();
                return true;
            } else {
                m_rgb_mutex.unlock();
                return false;
            }
        }

        bool getDepth(cv::Mat& output) {
                m_depth_mutex.lock();
                if(m_new_depth_frame) {
                    depthMat.copyTo(output);
                    m_new_depth_frame = false;
                    m_depth_mutex.unlock();
                    return true;
                } else {
                    m_depth_mutex.unlock();
                    return false;
                }
            }
    private:
        std::vector<uint8_t> m_buffer_depth;
        std::vector<uint8_t> m_buffer_rgb;
        std::vector<uint16_t> m_gamma;
        cv::Mat depthMat;
        cv::Mat rgbMat;
        cv::Mat ownMat;
        myMutex m_rgb_mutex;
        myMutex m_depth_mutex;
        bool m_new_rgb_frame;
        bool m_new_depth_frame;
};
#endif

/// Grabber implementation
class KinectGrabber : public Grabber {
    public:

        /// Pointer
        typedef std::unique_ptr<KinectGrabber> Ptr;

        /// Construction
        KinectGrabber(void);

        /// Construction
        KinectGrabber(std::string modelFilename, Mode _model) : Grabber("Kinect Grabber", TYPE_PRIMESENSE, _model), model(modelFilename),
        device(freenect.createDevice<MyFreenectDevice>(0)){
            device.startVideo();
            device.startDepth();
            usleep(1000000);
            cv::Mat depthMat(cv::Size(640,480),CV_16UC1);
            device.getDepth(depthMat);
            sensorFrame.depthImage = depthMat.clone();
            frameNo=0;

        }

        ///Destruction
        ~KinectGrabber()
        {
            device.stopVideo();
            device.stopDepth();
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

        const SensorFrame& getSensorFrame(void);

        /// Return starting position of sensor
        Eigen::Matrix4f getStartingSensorPose();

    private:
        Freenect::Freenect freenect;
        MyFreenectDevice& device;

        /// Sensor model
        DepthSensorModel model;
        int frameNo;

};

#endif // KINECT_GRABBER_H_INCLUDED
