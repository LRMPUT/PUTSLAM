// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
    #define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "../include/Grabber/xtion_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>


#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

using namespace putslam;

/// A single instance of Kinect grabber
XtionGrabber::Ptr grabberX;

XtionGrabber::XtionGrabber(void) : Grabber("Xtion Grabber", TYPE_PRIMESENSE) {
    rc = openni::STATUS_OK;
    initOpenNI();

}

XtionGrabber::XtionGrabber(std::string modelFilename) : Grabber("Xtion Grabber", TYPE_PRIMESENSE), model(modelFilename){
rc = openni::STATUS_OK;
initOpenNI();
}

int XtionGrabber::grabberClose(){

    depth.stop();
    color.stop();
    depth.destroy();
    color.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 0;
}


const std::string& XtionGrabber::getName() const {
    return name;
}

const PointCloud& XtionGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& XtionGrabber::getSensorFrame(void) const {

    printf("I'm in get sensor frame. Size of Matrices is: %d, %d, %d, %d\n",sensor_frame.depth.rows,sensor_frame.depth.cols,sensor_frame.image.rows,sensor_frame.image.cols);
    return sensor_frame;
}

int XtionGrabber::initOpenNI(){

    const char* deviceURI = openni::ANY_DEVICE;  //we only have one device any device

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }

    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = depth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!depth.isValid() || !color.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }

    rc = device.setDepthColorSyncEnabled(true);
    if (rc != openni::STATUS_OK) {
        printf("Couldn't enable depth and color images synchronization\n%s\n",openni::OpenNI::getExtendedError());
        return 2;
    }
}

int XtionGrabber::acquireDepthFrame(cv::Mat &m){

    rc = depth.readFrame(&m_depthFrame);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed\n");
        return 1;
    }

    if (m_depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM && m_depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        printf("Unexpected frame format\n");
    }


    openni::DepthPixel* pDepth = (openni::DepthPixel*)m_depthFrame.getData();
    m.create(m_depthFrame.getHeight(),m_depthFrame.getWidth(),CV_16SC1);  //floating point values for depth values
    memcpy(m.data,pDepth,m_depthFrame.getStrideInBytes() * m_depthFrame.getHeight());
    return 0;

}

int XtionGrabber::acquireColorFrame(cv::Mat &m){

    rc = color.readFrame(&m_colorFrame);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed\n");
        return 1;
    }

    //TODO
    //if (frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM)
    //{
    //    printf("Unexpected frame format\n");
    //}

    const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
    m.create(m_colorFrame.getHeight(),m_colorFrame.getWidth(),CV_8UC3);
    memcpy(m.data,pImageRow,m_colorFrame.getStrideInBytes() * m_colorFrame.getHeight());
    cv::cvtColor(m, m, CV_RGB2BGR);

    return 0;


}

void XtionGrabber::grab(void) {
//    Point3D point;
//    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
//    point.x = 1.2; point.y = 3.4; point.z = 5.6;
//    cloud.push_back(point);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
      printf("I'm in Xtion getSensorFrame\n");
      if(acquireDepthFrame(this->sensor_frame.depth)) throw 1;
      if(acquireColorFrame(this->sensor_frame.image)) throw 2;
}

/// run grabber thread
void XtionGrabber::calibrate(void) {

}

putslam::Grabber* putslam::createGrabberXtion(void) {
    grabberX.reset(new XtionGrabber());
    return grabberX.get();
}

putslam::Grabber* putslam::createGrabberXtion(std::string configFile) {
    grabberX.reset(new XtionGrabber(configFile));
    return grabberX.get();
}
