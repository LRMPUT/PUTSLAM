#include "../../include/putslam/Grabber/fileGrabber.h"

#include "Defs/opencv.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

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
	startPlayTimestamp = std::chrono::high_resolution_clock::now();
	startSeqTimestamp = -1.0;
	lastSeqTimestamp = -1;
	fileNo = -1;
	proccesingFileCounter = 0;

	timestampFile.open(parameters.fullPath + "matched");
}

FileGrabber::~FileGrabber(void) {
}

bool FileGrabber::grab(void) {
	// File was already fully read
	if (timestampFile.eof())
		return false;

	if ( parameters.verbosePlayParameters > 0)
		std::cout << "FileGrabber: proccesingFileCounter vs maxNumberOfFrames : " << proccesingFileCounter << " "
			<< parameters.maxNumberOfFrames << std::endl;
	// Check max number of frames
	if ( proccesingFileCounter >= parameters.maxNumberOfFrames) {
		return false;
	}

	// sensorFrame to read rgb image, depth image and timestamp
	SensorFrame tmpSensorFrame;
	tmpSensorFrame.depthImageScale = parameters.depthImageScale;


	double timestamp = 0;
	int previousLinePlace;

	// If the option to skip frames was activated (playEveryNth > 1), we skip some frames
	// If the real time option is activated we drop frames until we catch up current time
	for (int i = 0; (i < parameters.playEveryNth) || parameters.realTime ; i++) {

		// Increment file number
        fileNo++;

		// We save the position of last line in case we need to go back by one line
		previousLinePlace = (int) timestampFile.tellg();

		// Consider timestamps from provided file
		std::string timestampString;
		std::getline(timestampFile, timestampString);

		// No more timestamps ---> we detected the end of sequence
		if (timestampString.length() < 3)
			return false;

		// Compute the average of rgb and depth timestamps
		double timestamp1 = atof(
				timestampString.substr(0, timestampString.find(' ')).c_str());
		double timestamp2 = atof(
				timestampString.substr(timestampString.find(' ') + 1).c_str());
		timestamp = (timestamp1 + timestamp2) / 2;

		// In case of processing real time we check more conditions
		if ( parameters.realTime )
		{
			// Compute milliseconds between current frame and start of sequence
			double msSeq = timestamp - startSeqTimestamp;

			// Compute milliseconds between current time and start of processing
			double msPlay = (double) std::chrono::duration_cast < std::chrono::milliseconds
					> (std::chrono::high_resolution_clock::now()
							- startPlayTimestamp).count()/1000.0;

			if (parameters.verbosePlayParameters > 1)
				std::cout<<"Considering " << fileNo << ":\tplayTime=" << msPlay << " [s]\tsequenceTime=" << msSeq <<" [s]"<< std::endl;

			// Sequence time is ahead of processing or we just started
			if ( msPlay < msSeq || startSeqTimestamp< 0)
			{
				// we just started
				if (startSeqTimestamp < 0)
				{
					startSeqTimestamp = timestamp;
					startPlayTimestamp = std::chrono::high_resolution_clock::now();
				}
				// we can read one image earlier
				if (lastSeqTimestamp > 0)
				{
					fileNo--;
					lastSeqTimestamp = timestamp;
					timestampFile.seekg(previousLinePlace);
				}
				lastSeqTimestamp = timestamp;
				break;
			}
		}
	}

	if (parameters.verbosePlayParameters > 0)
		std::cout << "Measurement timestamp : " << convertToHighPrecisionString(timestamp) << std::endl;
	tmpSensorFrame.readId = fileNo;
	tmpSensorFrame.timestamp = timestamp;

	// RGB
    std::ostringstream oss;
	oss <<  std::setfill('0') << std::setw(5) << fileNo;
	if (parameters.verbosePlayParameters > 0)
		std::cout << "Loading file: " << "rgb_" + oss.str() << ".png" << std::endl;
    tmpSensorFrame.rgbImage = cv::imread(parameters.fullPath + "rgb_" +  oss.str() +".png", CV_LOAD_IMAGE_COLOR );
    if(!tmpSensorFrame.rgbImage.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
    }

    //Depth
    std::ostringstream ossDepth;
    ossDepth << std::setfill('0') << std::setw(5) <<  fileNo ;
    if (parameters.verbosePlayParameters > 0)
    	std::cout << "Loading file: " << "depth_" << ossDepth.str() << ".png" << std::endl;
    tmpSensorFrame.depthImage = cv::imread(parameters.fullPath + "depth_" + ossDepth.str() + ".png", CV_LOAD_IMAGE_ANYDEPTH );
    if(!tmpSensorFrame.rgbImage.data ) {
       std::cout <<  "Could not open or find the image" << std::endl ;
    }

    // New image had been processed
    proccesingFileCounter++;

    // Add to queue
    mtx.lock();
    if (mode==MODE_CONTINUOUS)
        sensorFrame = tmpSensorFrame;
    else if (mode==MODE_BUFFER){
        sensorFrames.push(tmpSensorFrame);
    }
    mtx.unlock();

    return true;
}

std::string FileGrabber::convertToHighPrecisionString(double timestamp, int precision) {
	std::ostringstream ossTimestamp;
	ossTimestamp << imageSeqPrefix << std::setfill('0') << std::setprecision(precision)
			<< timestamp;
	return ossTimestamp.str();
}

/// Set sequence properties
void FileGrabber::setSequence(const uint_fast32_t startFrameNo, const std::string& imagePrefix, const std::string& depthPrefix, const std::string& cloudPrefix){
    imageSeqPrefix = imagePrefix;
    depthSeqPrefix = depthPrefix;
    cloudSeqPrefix = cloudPrefix;
    fileNo = (int) startFrameNo;
}

/// Grab sequence of image and save sequence to files (duration in number of frames)
void FileGrabber::getSequence(const uint_fast32_t duration){
    //sensorFrames.clear();
    for (size_t i=0;i<duration;i++){ //recording
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
	float x, y, z, qw, qx, qy, qz;
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
