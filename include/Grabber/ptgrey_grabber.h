/** @file ptgrey_grabber.h
 *
 * implementation - Ptgrey Grabber
 *
 */

#ifndef PTGREY_GRABBER_H_INCLUDED
#define PTGREY_GRABBER_H_INCLUDED

#include "grabber.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>
#ifdef WITH_PTGREY
    #include "FlyCapture2.h"
#endif
#include <chrono>
#include <thread>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifdef WITH_PTGREY
    using namespace FlyCapture2;
#endif


namespace putslam {
    /// create a single grabber (Ptgrey)
    Grabber* createGrabberPtgrey(void);
    Grabber* createGrabberPtgrey(std::string configFile, Grabber::Mode mode);
};

using namespace putslam;

/// Grabber implementation
class PtgreyGrabber : public Grabber {
    public:

        /// Pointer
        typedef std::unique_ptr<PtgreyGrabber> Ptr;

        /// Construction
        PtgreyGrabber(void);

        /// Construction
        PtgreyGrabber(std::string modelFilename, Mode _mode);

        /// Name of the grabber
        virtual const std::string& getName() const;

        /// Returns current point cloud
        virtual const PointCloud& getCloud(void) const;

        /// Grab image and/or point cloud
        virtual bool grab();
    #ifdef WITH_PTGREY
        ///Sensor initialization
        virtual int initPtGrey ();
    #endif
        /// Calibrate sensor
        virtual void calibrate(void);

        ///Sensor uninitialize
        virtual int grabberClose(void);

        /// Return starting position of sensor TODO: Implement
        Eigen::Matrix4f getStartingSensorPose(){};

    protected:
    #ifdef WITH_PTGREY
        Error error;
        BusManager busMgr;
        unsigned int numCameras;
        PGRGuid guid;
        Camera cam;

    private:
        ///Prints out error trace
        void PrintError( Error error );
        ///Prints out camera info
        void PrintCameraInfo( CameraInfo* pCamInfo );
    #endif

};

#endif // PTGREY_GRABBER_H_INCLUDED
