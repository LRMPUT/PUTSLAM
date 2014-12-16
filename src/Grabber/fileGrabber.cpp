#include "../include/Grabber/fileGrabber.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>

using namespace putslam;
using namespace std::chrono;

/// A single instance of file grabber
FileGrabber::Ptr fileGrabber;

FileGrabber::FileGrabber(void) : Grabber("File Grabber", TYPE_RGB), asusModel("../resources/KinectModel.xml") {
    startT = std::chrono::high_resolution_clock::now();
}

FileGrabber::~FileGrabber(void) {
}

bool FileGrabber::grab(void) {
    std::ostringstream oss;
    oss << imageSeqPrefix << std::setfill('0') << std::setw(5) << fileNo << ".png";
    std::cout << "loadFile:" << oss.str() << std::endl;
    SensorFrame tmp;
    tmp.image = cv::imread( oss.str(), CV_LOAD_IMAGE_COLOR );
    if(!tmp.image.data ) {// Check for invalid input
       // std::cout <<  "Could not open or find the image" << std::endl ;
       // return false;
    }
    std::ostringstream ossDepth;
    ossDepth << depthSeqPrefix << std::setfill('0') << std::setw(5) <<  fileNo << ".png";
    tmp.depth = cv::imread( ossDepth.str(), CV_LOAD_IMAGE_COLOR );
    std::cout << "loadFile:" << ossDepth.str() << std::endl;
    if(!tmp.image.data ) {// Check for invalid input
        //std::cout <<  "Could not open or find the image" << std::endl ;
        //return false;
    }
    std::ostringstream ossCloud;
    ossCloud << cloudSeqPrefix << std::setfill('0') << std::setw(5) <<  fileNo << ".pcd";
    //pcl::io::loadPCDFile( ossCloud.str(), tmp.cloud);
    std::cout << "loadFile:" << ossCloud.str() << std::endl;
    /*if(!tmp.cloud.size()>0 ) {// Check for invalid input
        std::cout <<  "Could not open or find the cloud" << std::endl ;
        return false;
    }*/
    //SensorFrame frameTmp;
    //frameTmp.depth = tmp.depth.clone();
    //tmp.depth.convertTo(tmp.depth, CV_16SC1);
    //tmp.cloud = asusModel.depth2cloud(tmp.depth);
    tmp.timestamp = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now() - startT).count();
    mtx.lock();
    sensorFrames.push(tmp);
    mtx.unlock();
    fileNo++;
    return true;
}

/// Set sequence properties
void FileGrabber::setSequence(const uint_fast32_t startFrameNo, const std::string& imagePrefix, const std::string& depthPrefix, const std::string& cloudPrefix){
    imageSeqPrefix = imagePrefix;
    depthSeqPrefix = depthPrefix;
    cloudSeqPrefix = cloudPrefix;
    fileNo = startFrameNo;
}

/// Grab sequence of image and save sequence to files (duration in number of frames)
void FileGrabber::getSequence(const uint_fast32_t duration){
    //sensorFrames.clear();
    for (int i=0;i<duration;i++){ //recording
        grab(); // grab frame
        //sensorFrames.push(sensorFrame);
    }
}

/// run grabber thread
void FileGrabber::calibrate(void) {

}

/// Name of the grabber
const std::string& FileGrabber::getName() const {
    return name;
}

/// Returns current frame
const SensorFrame& FileGrabber::getSensorFrame(void) {
    bool waitForImage = true;
    while (waitForImage){
        mtx.lock();
        if (sensorFrames.size())
            waitForImage = false;
        mtx.unlock();
    }
    mtx.lock();
    sensorFrame = sensorFrames.front();
    sensorFrames.pop();
    mtx.unlock();
    return sensorFrame;
}

/// Returns the current point cloud
const PointCloud& FileGrabber::getCloud(void) const{
    return cloud;
}

///Clossing a device
int FileGrabber::grabberClose() {
    return 0;
}

putslam::Grabber* putslam::createFileGrabber(void) {
    fileGrabber.reset(new FileGrabber());
    return fileGrabber.get();
}
