#include "../../include/Grabber/fileGrabber.h"

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

FileGrabber::FileGrabber(void) : Grabber("File Grabber", TYPE_RGB, MODE_BUFFER) {
	initFileGrabber();
}

FileGrabber::FileGrabber(std::string configFilename) : Grabber("File Grabber", TYPE_PRIMESENSE, MODE_BUFFER), parameters(configFilename){
	initFileGrabber();
}

void FileGrabber::initFileGrabber() {
	startT = std::chrono::high_resolution_clock::now();

	std::cout << "Open : " << parameters.fullPath + "matched" << std::endl;
	timestampFile.open(parameters.fullPath + "matched");
}

FileGrabber::~FileGrabber(void) {
}

bool FileGrabber::grab(void) {
	// Some variables
	SensorFrame tmp;

	std::string timestampString;
	// If the option to skip frames was activated (playEveryNth > 1), we skip some frames
	for (int i = 0; i < parameters.playEveryNth; i++) {
		// Increment file number
		fileNo++;

		// Consider timestamps from provided file
		std::getline(timestampFile, timestampString);
	}

	// End of file
	if( timestampString.length() < 3)
		return false;

	// Compute the average of rgb and depth timestamp
	double timestamp1 = atof(
			timestampString.substr(0, timestampString.find(' ')).c_str());
	double timestamp2 = atof(
			timestampString.substr(timestampString.find(' ') + 1).c_str());

	std::ostringstream ossTimestamp;
	ossTimestamp << imageSeqPrefix << std::setfill('0') << std::setprecision(17)
			<< (timestamp1 + timestamp2) / 2;
	std::cout << "Measurement timestamp : " << ossTimestamp.str() << std::endl;
	tmp.timestamp = (timestamp1 + timestamp2) / 2;

	// RGB
	std::ostringstream oss;
	oss << imageSeqPrefix << std::setfill('0') << std::setw(5) << fileNo;
    std::cout << "Loading file: " << "rgb_" + oss.str() << ".png" << std::endl;
    tmp.image = cv::imread(parameters.fullPath + "rgb_" +  oss.str() +".png", CV_LOAD_IMAGE_COLOR );
    if(!tmp.image.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
    }

    //Depth
    std::ostringstream ossDepth;
    ossDepth << depthSeqPrefix << std::setfill('0') << std::setw(5) <<  fileNo ;
    std::cout << "Loading file: " << "depth_" << ossDepth.str() << ".png" << std::endl;
    tmp.depth = cv::imread(parameters.fullPath + "depth_" + ossDepth.str() + ".png", CV_LOAD_IMAGE_ANYDEPTH );
    if(!tmp.image.data ) {
       std::cout <<  "Could not open or find the image" << std::endl ;
    }



    // Add to queue
    mtx.lock();
    if (mode==MODE_CONTINUOUS)
        sensorFrame = tmp;
    else if (mode==MODE_BUFFER){
        sensorFrames.push(tmp);
    }
    mtx.unlock();

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

/// Return starting position of sensor
Eigen::Matrix4f FileGrabber::getStartingSensorPose()
{
	std::ifstream initialSensorPoseStream(parameters.fullPath + "initialPosition");
	double x, y, z, qw, qx, qy, qz;
	initialSensorPoseStream >> x >> y >> z >> qx >> qy >> qz >> qw;
	initialSensorPoseStream.close();

	Eigen::Matrix4f initialSensorPose = Eigen::Matrix4f::Identity();
	Eigen::Quaternion<float> quat(qw, qx, qy, qz);
	initialSensorPose(0, 3) = x;
	initialSensorPose(1, 3) = y;
	initialSensorPose(2, 3) = z;
	initialSensorPose.block<3, 3>(0, 0) = quat.toRotationMatrix();
	return initialSensorPose;
}

putslam::Grabber* putslam::createGrabberFile(void) {
    fileGrabber.reset(new FileGrabber());
    return fileGrabber.get();
}

putslam::Grabber* putslam::createGrabberFile(std::string configFile) {
    fileGrabber.reset(new FileGrabber(configFile));
	return fileGrabber.get();
}
