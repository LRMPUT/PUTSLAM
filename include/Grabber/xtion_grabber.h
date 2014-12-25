/** @file xtion_grabber.h
 *
 * implementation - Xtion Grabber
 *
 */
#ifndef XTION_GRABBER_H
#define XTION_GRABBER_H

#include <stddef.h>
#include <OpenNI.h>
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "grabber.h"
#include "depthSensorModel.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

#define MAX_DEPTH 10000

enum DisplayModes
{
    DISPLAY_MODE_OVERLAY,
    DISPLAY_MODE_DEPTH,
    DISPLAY_MODE_IMAGE
};

namespace putslam {
    /// create a single grabber (Xtion)
    Grabber* createGrabberXtion(void);
    Grabber* createGrabberXtion(std::string configFile, Grabber::Mode mode);
};

using namespace putslam;

class XtionGrabber : public Grabber {

    public:

    /// Pointer
    typedef std::unique_ptr<XtionGrabber> Ptr;

    /// Construction
    XtionGrabber(void);

    /// Construction
    XtionGrabber(std::string modelFilename, Mode _mode);

    /// Name of the grabber
    virtual const std::string& getName() const;

    /// Returns current point cloud
    virtual const PointCloud& getCloud(void) const;

    /// Grab image and/or point cloud
    virtual bool grab();

    /// Calibrate sensor
    virtual void calibrate(void);

    //local functions

    ///Sensor initialization
    virtual int initOpenNI ();

    ///Sensor uninitialize
    virtual int grabberClose();

    DepthSensorModel model;

protected:

    //Comments on variables
    openni::Status rc;
    openni::Device device;
    openni::VideoStream depth;
    openni::VideoStream color;
    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;
    const openni::SensorInfo *depthSensorInfo;
    const openni::SensorInfo *colorSensorInfo;

    int depthMode;
    int colorMode;
    bool syncDepthColor;
    ///Acquisition of Depth Frame
    int acquireDepthFrame(cv::Mat &m);
    ///Acquisition of Color Frame
    int acquireColorFrame(cv::Mat &m);
    ///Lists all the Depth Video Modes available for current sensor
    int listDepthVideoMode();
    ///Lists all the Color Video Modes available for current sensor
    int listColorVideoMode();


private:


};


#endif // XTION_GRABBER_H
