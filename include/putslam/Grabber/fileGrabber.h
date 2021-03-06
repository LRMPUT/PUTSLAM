/** @file fileGrabber.h
 *
 * implementation - Grabber loads images from file
 *
 */

#ifndef FILE_GRABBER_H_INCLUDED
#define FILE_GRABBER_H_INCLUDED

#include "grabber.h"
#include "depthSensorModel.h"
#include "Grabber/grabber.h"
#include "Defs/opencv.h"
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
					"verbose", &verbosePlayParameters);
			config.FirstChildElement("playParameters")->QueryIntAttribute(
					"playEveryNthFrame", &playEveryNth);
			config.FirstChildElement("playParameters")->QueryBoolAttribute(
					"realTime", &realTime);
			config.FirstChildElement("playParameters")->QueryIntAttribute(
					"maxNumberOfFrames", &maxNumberOfFrames);


			if ( verbosePlayParameters > 0) {
				std::cout<<"File grabber play parameters: " << std::endl;
				std::cout<<"\t verbose = " << verbosePlayParameters << std::endl;
				std::cout<<"\t playEveryNthFrame = " << playEveryNth << std::endl;
				std::cout<<"\t realTime = " << realTime << std::endl;
				std::cout<<"\t maxNumberOfFrames = " << maxNumberOfFrames << std::endl;

				if ( verbosePlayParameters > 1) {
					getchar();
				}
			}

			// Get dataset config
			std::string datasetCfgFilename =
					config.FirstChildElement("Model")->Attribute("datasetFile");
			config.FirstChildElement("Model")->QueryIntAttribute("verbose", &verboseModel);
			tinyxml2::XMLDocument datasetCfg;
			filename = "../../resources/" + datasetCfgFilename;
			datasetCfg.LoadFile(filename.c_str());
			if (datasetCfg.ErrorID()) {
				std::cout << "Unable to load dataset config file: "
						<< configFilename << std::endl;
			}




			// dataset path
			tinyxml2::XMLElement *params = datasetCfg.FirstChildElement("datasetPath");
			basePath = params->Attribute("base");
			datasetName = params->Attribute("datasetName");
			fullPath = basePath + "/" + datasetName + "/";

			params->QueryDoubleAttribute("depthImageScale", &depthImageScale);

			if ( verboseModel > 0) {
				std::cout<<"File grabber model: " << std::endl;
				std::cout<<"\t depthImageScale = " << depthImageScale << std::endl;
				std::cout<<"\t fullPath = " << fullPath << std::endl;

				if ( verboseModel > 1) {
					getchar();
				}
			}

		}
	public:
		 /// path of the dataset
		 std::string basePath, datasetName, fullPath;

		 /// DepthImageScale
		 double depthImageScale;

		 /// Play parameters
		 int playEveryNth;
		 bool realTime;

		 // Maximal number of frames
		 int maxNumberOfFrames;

		 /// Verbose
		 int verbosePlayParameters;
		 int verboseModel;
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
