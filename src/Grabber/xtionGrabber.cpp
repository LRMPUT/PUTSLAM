
// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
    #define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "../../include/Grabber/xtionGrabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>


#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

using namespace putslam;

/// A single instance of Kinect grabber
XtionGrabber::Ptr grabberX;

XtionGrabber::XtionGrabber(void) : Grabber("Xtion Grabber", TYPE_PRIMESENSE, MODE_BUFFER) {
    rc = openni::STATUS_OK;
    initOpenNI();

}

XtionGrabber::XtionGrabber(std::string modelFilename, Mode _mode) : Grabber("Xtion Grabber", TYPE_PRIMESENSE, _mode), model(modelFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + modelFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Xtion config file.\n";
    else{
        tinyxml2::XMLElement * xtionDevice = config.FirstChildElement( "xtionDevice" );
        xtionDevice->FirstChildElement( "depthVideoMode" )->QueryIntText(&depthMode);
        xtionDevice->FirstChildElement( "colorVideoMode" )->QueryIntText(&colorMode);
        xtionDevice->FirstChildElement("depthColorSyncEnabled")->QueryBoolText(&syncDepthColor);
    }
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
    if(syncDepthColor){
        rc = device.setDepthColorSyncEnabled(true);
        if (rc != openni::STATUS_OK) {
            printf("Couldn't enable depth and color images synchronization\n%s\n",openni::OpenNI::getExtendedError());
            return 2;
        }
    }
    listDepthVideoMode();
    rc = depth.setVideoMode(depthSensorInfo->getSupportedVideoModes()[depthMode]); //best option 4
    if (rc != openni::STATUS_OK) {
        printf("Couldn't set proper Depth Video Mode\n%s\n",openni::OpenNI::getExtendedError());
        return 2;
    }
    //Importand otherwise error !!!
    //required pause for about 1 second to properly set the Video Modes, otherwise core dump error
    listColorVideoMode(); //this line gives a required pause
    rc = color.setVideoMode(colorSensorInfo->getSupportedVideoModes()[colorMode]); //best option 9
    if (rc != openni::STATUS_OK) {
        printf("Couldn't set proper Color Video Mode\n%s\n",openni::OpenNI::getExtendedError());
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
    mtx.lock();
    m.create(m_depthFrame.getHeight(),m_depthFrame.getWidth(),CV_16UC1);  //floating point values for depth values. Important -- use 16UC1 in order to properly store data in png file.
    memcpy(m.data,pDepth,m_depthFrame.getStrideInBytes() * m_depthFrame.getHeight());
    mtx.unlock();
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

    mtx.lock();
    m.create(m_colorFrame.getHeight(),m_colorFrame.getWidth(),CV_8UC3);
    memcpy(m.data,pImageRow,m_colorFrame.getStrideInBytes() * m_colorFrame.getHeight());
    cv::cvtColor(m, m, CV_RGB2BGR);
    mtx.unlock();
    return 0;


}

bool XtionGrabber::grab(void) {
//    Point3D point;
//    point.r = 255; point.g = 0; point.b = 0; point.a = 255;
//    point.x = 1.2; point.y = 3.4; point.z = 5.6;
//    cloud.push_back(point);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
      mtx.lock();
      if(acquireDepthFrame(this->sensorFrame.depth)) throw 1;
      if(acquireColorFrame(this->sensorFrame.image)) throw 2;
      else if (mode==MODE_BUFFER) {
          sensorFrames.push(sensorFrame);
      }
      mtx.unlock();
      return true;
}

int XtionGrabber::listDepthVideoMode(){

    depthSensorInfo = device.getSensorInfo(openni::SENSOR_DEPTH);
    for(int i=0;i < depthSensorInfo->getSupportedVideoModes().getSize();i++)
        {
            openni::VideoMode videoMode = depthSensorInfo->getSupportedVideoModes()[i];

            std::cout << i<< ". fps: " << videoMode.getFps() << "x: " << videoMode.getResolutionX() << "y " <<  videoMode.getResolutionY() << std::endl;
        }

    return 0;
}

int XtionGrabber::listColorVideoMode(){

    colorSensorInfo = device.getSensorInfo(openni::SENSOR_COLOR);
    for(int i=0;i < colorSensorInfo->getSupportedVideoModes().getSize();i++)
        {
            openni::VideoMode videoMode = colorSensorInfo->getSupportedVideoModes()[i];

            std::cout << i<< ". fps: " << videoMode.getFps() << "x: " << videoMode.getResolutionX() << "y " <<  videoMode.getResolutionY() << std::endl;
        }

    return 0;
}


/// run grabber thread
void XtionGrabber::calibrate(void) {

}

Eigen::Matrix4f XtionGrabber::getStartingSensorPose()
{
	Eigen::Matrix4f::Identity();
}

putslam::Grabber* putslam::createGrabberXtion(void) {
    grabberX.reset(new XtionGrabber());
    return grabberX.get();
}

putslam::Grabber* putslam::createGrabberXtion(std::string configFile, Grabber::Mode mode) {
    grabberX.reset(new XtionGrabber(configFile, mode));
    return grabberX.get();
}

