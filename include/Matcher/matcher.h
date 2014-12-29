/** @file matcher.h
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#ifndef _MATCHER_H_
#define _MATCHER_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>
#include "opencv/cv.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "../TransformEst/RANSAC.h"

namespace putslam {
/// Grabber interface
class Matcher {
public:
	struct parameters {
			std::string detector;
			std::string descriptor;
		};

	/// Overloaded constructor
	Matcher(const std::string _name) :
			name(_name), frame_id(0) {
	}
	Matcher(const std::string _name, const std::string parametersFile) :
			name(_name), frame_id(0), matcherParameters(parametersFile) {
	}

	/// Name of the Matcher
	virtual const std::string& getName() const = 0;

	/// Load features at the start of the sequence
	void loadInitFeatures(const SensorFrame& sensorData);

	/// Run single match
	bool match(const SensorFrame& sensorData, Eigen::Matrix4f &estimatedTransformation);

	/// Class used to hold all parameters
	class Parameters {
	public:
		Parameters(){};
		Parameters(std::string configFilename) {
			tinyxml2::XMLDocument config;
			std::string filename = "../../resources/" + configFilename;
			config.LoadFile(filename.c_str());
			if (config.ErrorID())
			{
				std::cout << "Unable to load Matcher OpenCV config file: " << configFilename << std::endl;
			}
			tinyxml2::XMLElement * params = config.FirstChildElement("Matcher");
			// Matcher
			params->QueryIntAttribute("verbose", &verbose);
			// RANSAC
			params->FirstChildElement("RANSAC")->QueryIntAttribute("verbose", &RANSACParams.verbose);
			params->FirstChildElement("RANSAC")->QueryDoubleAttribute("inlierThreshold",
					&RANSACParams.inlierThreshold);
			params->FirstChildElement("RANSAC")->QueryIntAttribute("usedPairs", &RANSACParams.usedPairs);
			// Matcher OpenCV
			OpenCVParams.detector = params->FirstChildElement("MatcherOpenCV")->Attribute("detector");
			OpenCVParams.descriptor = params->FirstChildElement("MatcherOpenCV")->Attribute("descriptor");

		}
	public:
		int verbose;
		RANSAC::parameters RANSACParams;
		Matcher::parameters OpenCVParams;
	};


protected:

	/// Matcher name
	const std::string name;

	/// Frame id
	uint_fast32_t frame_id;

	/// Information about previous keypoints + descriptors
	std::vector<cv::KeyPoint> prevFeatures;
	cv::Mat prevDescriptors;
	std::vector<Eigen::Vector3f> prevFeatures3D;
	cv::Mat prevRgbImage, prevDepthImage;

	/// Parameters
	Parameters matcherParameters;

	/// Methods used to visualize results/data
	void showFeatures(cv::Mat rgbImage, std::vector<cv::KeyPoint> features);
	void showMatches(cv::Mat prevRgbImage,
			std::vector<cv::KeyPoint> prevFeatures, cv::Mat rgbImage,
			std::vector<cv::KeyPoint> features, std::vector<cv::DMatch> matches);

	/// Detect features
	virtual std::vector<cv::KeyPoint> detectFeatures(cv::Mat rgbImage) = 0;

	/// Describe features
	virtual cv::Mat describeFeatures(cv::Mat rgbImage,
			std::vector<cv::KeyPoint> features) = 0;

	/// Perform matching
	virtual std::vector<cv::DMatch> performMatching(cv::Mat prevDescriptors,
			cv::Mat descriptors) = 0;


};
};

#endif // _MATCHER_H_
