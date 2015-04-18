/** @file fileGrabber.h
 *
 * implementation - Grabber loads images from file
 *
 */

#ifndef FILE_GRABBER_H_INCLUDED
#define FILE_GRABBER_H_INCLUDED

#include "grabber.h"
#include "depthSensorModel.h"
#include "../include/Grabber/grabber.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <chrono>
#include <memory>
#include <fstream>
#include <stdlib.h>

namespace putslam {
    /// create a single grabber (Generic Camera)
    Grabber* createGrabberFile(void);
    Grabber* createGrabberFile(std::string configFile);
};

using namespace putslam;

/// Grabber implementation
class FileGrabber : public Grabber {
    public:
        /// Pointer
        typedef std::unique_ptr<FileGrabber> Ptr;

        /// Construction
        FileGrabber(void);
        FileGrabber(std::string configFilename);

        ///
        void initFileGrabber();

        /// Destructor
        ~FileGrabber(void);

        /// Grab image and/or point cloud
        bool grab();

        /// Set sequence properties
        void setSequence(const uint_fast32_t startFrameNo, const std::string& imagePrefix, const std::string& depthPrefix, const std::string& cloudPrefix);

        /// Grab sequence of image and save sequence to files (duration in number of frames)
        void getSequence(const uint_fast32_t duration);

        /// Calibrate sensor
        void calibrate(void);

        /// Name of the grabber
        const std::string& getName() const;

        /// Returns current frame
        const SensorFrame& getSensorFrame(void);

        /// Returns the current point cloud
        const PointCloud& getCloud(void) const;

        /// Closing a device
        int grabberClose();

        /// Return starting position of sensor
        Eigen::Matrix4f getStartingSensorPose();

        /// Class used to hold all parameters
	class Parameters {
	public:
		Parameters() {
		}
		;
		Parameters(std::string configFilename) {
			tinyxml2::XMLDocument config;
			std::string filename = "../../resources/" + configFilename;
			config.LoadFile(filename.c_str());
			if (config.ErrorID()) {
				std::cout << "Unable to load File Grabber config file: "
						<< configFilename << std::endl;
			}


			// Play parameters
			config.FirstChildElement("playParameters")->QueryIntAttribute(
					"verbose", &verbose);
			config.FirstChildElement("playParameters")->QueryIntAttribute(
					"playEveryNthFrame", &playEveryNth);
			config.FirstChildElement("playParameters")->QueryBoolAttribute(
					"realTime", &realTime);
			config.FirstChildElement("playParameters")->QueryIntAttribute(
					"maxNumberOfFrames", &maxNumberOfFrames);

			// dataset path
			tinyxml2::XMLElement *params = config.FirstChildElement("datasetPath");
			basePath = params->Attribute("base");
			datasetName = params->Attribute("datasetName");
			fullPath = basePath + "/" + datasetName + "/";
		}
	public:
		 /// path of the dataset
		 std::string basePath, datasetName, fullPath;

		 /// Play parameters
		 int playEveryNth;
		 bool realTime;

		 // Maximal number of frames
		 int maxNumberOfFrames;

		 /// Verbose
		 int verbose;
	};

    private:

		// Convert to string with high numer of digits
		std::string convertToHighPrecisionString(double timestamp, int precision = 20);

    public:
		/// Parameters read from file
		Parameters parameters;

    private:
        /// file prefix (images)
        std::string imageSeqPrefix;

        /// file prefix (depth images)
        std::string depthSeqPrefix;

        /// file prefix (point clouds)
        std::string cloudSeqPrefix;

        /// file number
        int fileNo, proccesingFileCounter;

        /// timestamp file
        std::ifstream timestampFile;

        /// timestamp at the start
        std::chrono::high_resolution_clock::time_point startPlayTimestamp;
        double startSeqTimestamp;
        double lastSeqTimestamp;


};

#endif // FILE_GRABBER_H_INCLUDED
