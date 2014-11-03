#include "../include/Grabber/ptgrey_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

using namespace putslam;

/// A single instance of Kinect grabber
PtgreyGrabber::Ptr grabberP;

PtgreyGrabber::PtgreyGrabber(void) : Grabber("Ptgrey Grabber", TYPE_PRIMESENSE) {

}

PtgreyGrabber::PtgreyGrabber(std::string modelFilename) : Grabber("Ptgrey Grabberr", TYPE_PRIMESENSE), model(modelFilename){
#ifdef WITH_PTGREY
    initPtGrey ();
#endif
}

const std::string& PtgreyGrabber::getName() const {
    return name;
}

const PointCloud& PtgreyGrabber::getCloud(void) const {
    return cloud;
}

const SensorFrame& PtgreyGrabber::getSensorFrame(void) const {
    return sensor_frame;
}

void PtgreyGrabber::grab(void) {
#ifdef WITH_PTGREY
    //RunSingleCamera( guid );

    Image rawImage;
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
        }

        printf( "Grabbed image\n");

        // Create a converted image
        Image convertedImage;

        // Convert the raw image
        error = rawImage.Convert( PIXEL_FORMAT_BGR, &convertedImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
        }

        this->sensor_frame.image.create(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3);
        memcpy(sensor_frame.image.data,convertedImage.GetData(),convertedImage.GetStride() * convertedImage.GetRows());
#endif
}

/// run grabber thread
void PtgreyGrabber::calibrate(void) {

}

int PtgreyGrabber::grabberClose(){
#ifdef WITH_PTGREY
    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
#endif
    return 0;
}

#ifdef WITH_PTGREY
void PtgreyGrabber::PrintError( Error error )
{
    error.PrintErrorTrace();
}

void PtgreyGrabber::PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}




int PtgreyGrabber::initPtGrey (){
    for(int i = 0; i<5; i++){
        error = busMgr.GetNumOfCameras(&numCameras);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        printf( "Number of cameras detected: %u\n", numCameras );
        if(!numCameras){
         error = busMgr.RescanBus();
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
    error = busMgr.RescanBus();
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
        }
        else break;
    }
    for (unsigned int i=0; i < numCameras; i++)
    {
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
    }
    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    std::chrono::milliseconds timespan(10000); // or whatever
    std::this_thread::sleep_for(timespan);
    PrintCameraInfo(&camInfo);
    // Start capturing images
    if(cam.IsConnected()){
    error = cam.StartCapture();
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }
    }
    else{
    printf( "Camera not connected\n");
     return -1;
    }

    return 0;
}
#endif

putslam::Grabber* putslam::createGrabberPtgrey(void) {
    grabberP.reset(new PtgreyGrabber());
    return grabberP.get();
}

putslam::Grabber* putslam::createGrabberPtgrey(std::string configFile) {
    grabberP.reset(new PtgreyGrabber(configFile));
    return grabberP.get();
}


