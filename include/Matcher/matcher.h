/** @file matcher.h
 *
 * \brief The core of matching Visual Odometry
 * \author Michal Nowicki
 *
 */

#ifndef _MATCHER_H_
#define _MATCHER_H_

#include "../Defs/putslam_defs.h"
#include "../Defs/opencvCore.h"
#include <string>
#include <vector>
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "../TransformEst/RANSAC.h"
#include "../include/RGBD/RGBD.h"

#include "MatchingOnPatches.h"
#include "chrono"

namespace putslam {
/// Grabber interface
class Matcher {

    static constexpr double scaleFactor = 1.2;
	static constexpr int nLevels = 8;
    static constexpr double logScaleFactor = std::log(scaleFactor);

public:
	struct featureSet {
		std::vector<cv::KeyPoint> feature2D;
        std::vector<double> detDist;
		std::vector<cv::Point2f> undistortedFeature2D, distortedFeature2D;
		cv::Mat descriptors;
		std::vector<Eigen::Vector3f> feature3D;
	};

	struct parameters {
		std::string detector;
		std::string descriptor;

		int gridRows, gridCols;

		int useInitialFlow;
		int winSize;
		int maxLevels;
		int maxIter;
		float eps;

		int minimalTrackedFeatures, maximalTrackedFeatures;
		double minimalReprojDistanceNewTrackingFeatures;
		double minimalEuclidDistanceNewTrackingFeatures;
		double DBScanEps;

		double matchingXYZSphereRadius;
		double matchingXYZacceptRatioOfBestMatch;

		double trackingErrorThreshold, trackingMinEigThreshold;
		int trackingErrorType;

		int removeTooCloseFeatures;
	};


	class TimeMeasurement {
		std::vector<long> times;

	public:
		double getDetectionAverage() {
			long sumTime = 0;
			for (auto &x : times) {
				sumTime += x;
			}
			return double(sumTime)/double(times.size());
		}

		void addTime(long time) {
			times.push_back(time);
		}
	};


	/// Overloaded constructor
	Matcher(const std::string _name) :
			name(_name), frame_id(0) {
		prevDescriptors = cv::Mat();
	}
	Matcher(const std::string _name, const std::string parametersFile,
			const std::string grabberParametersFile) :
			name(_name), frame_id(0), matcherParameters(parametersFile,
					grabberParametersFile) {
		prevDescriptors = cv::Mat();
	}

	~Matcher() {
	}

	/// Name of the Matcher
	virtual const std::string& getName() const = 0;

	/// Load features at the start of the sequence
	void detectInitFeatures(const SensorFrame& sensorData);

	/// Get current set of features
	Matcher::featureSet getFeatures();

	/// compute normals to rgbd features
	template<class T>
    void computeNormals(const cv::Mat& depthImage, T& features, double depthImageScale) {
        RGBD::computeNormals(depthImage, features,
                matcherParameters.cameraMatrixMat, depthImageScale);
	}

	/// compute RGB gradients to rgbd features
	template<class T>
	void computeRGBGradients(const cv::Mat& rgbImage, const cv::Mat& depthImage,
            T& features, double depthImageScale) {
        RGBD::computeRGBGradients(rgbImage, depthImage, features,
                matcherParameters.cameraMatrixMat, depthImageScale);
	}

	// VO
	double runVO(const SensorFrame& currentSensorFrame,
			Eigen::Matrix4f &estimatedTransformation,
			std::vector<cv::DMatch> &inlierMatches);

	/// Rung single KLT tracking
	double trackKLT(const SensorFrame& sensorData,
			Eigen::Matrix4f &estimatedTransformation,
			std::vector<cv::DMatch> &inlierMatches);

	/// Run single match
	double match(const SensorFrame& sensorData,
			Eigen::Matrix4f &estimatedTransformation,
			std::vector<cv::DMatch> &foundInlierMatches);

	/// Run the match with map
	double match(std::vector<MapFeature> mapFeatures, int sensorPoseId,
			std::vector<MapFeature> &foundInlierMapFeatures,
			Eigen::Matrix4f &estimatedTransformation);



	/// Run the match with map considering feature map location
	/// More like guided-matching
	double matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
			std::vector<MapFeature> &foundInlierMapFeatures,
			Eigen::Matrix4f &estimatedTransformation,
			bool newDetection,
			std::vector<int> frameIds = std::vector<int>());

	// Matching performed when loop closure is performed on features
	double matchFeatureLoopClosure(std::vector<MapFeature> featureSets[2], int framesIds[2], std::vector<std::pair<int, int>> &pairedFeatures,
			Eigen::Matrix4f &estimatedTransformation);

	int getNumberOfFeatures();


//	/// Run the match with two poses from the map. Parameters:
//	/// 	featureSet[2] 	-> set of features from the first and the second pose.
//	///			Remark: It only makes sense when features are moved to the local
//	///					coordinate system of features before calling the function
//	///		frameIds[2] 	-> set of ids of poses for features with the closest angle of observations -
//	///					in other words for each descriptor, we get the id of the pose so we can take the apprioprate descriptor
//	/// 	pairedFeatures 	-> return pairs of ids of features that are matched
//	///		estimatedTransformation		-> the estimated transformation
//	double matchPose2Pose(std::vector<MapFeature> featureSet[2],
//			std::vector<int> frameIds[2],
//			std::vector<std::pair<int, int>> &pairedFeatures,
//			Eigen::Matrix4f &estimatedTransformation);
//
//	/// Version as above, but for each feature it uses only the original descriptor
//    double matchPose2Pose(std::vector<MapFeature> featureSet[2],
//				std::vector<std::pair<int, int>> &pairedFeatures,
//				Eigen::Matrix4f &estimatedTransformation);
//
//    // Versions as above, but works on two sensor frames (it performs detection on its own)
//    double matchPose2Pose(SensorFrame sensorFrames[2],
//    		Eigen::Matrix4f &estimatedTransformation);
//
	/// Class used to hold all parameters
	class MatcherParameters {
	public:
		enum VOVERSION {
			VO_MATCHING, VO_TRACKING
		};
		enum MAPMATCHINGVERSION {
			MAPMATCH_DESCRIPTORS,
			MAPMATCH_XYZ_DESCRIPTORS,
			MAPMATCH_PATCHES,
			MAPMATCH_XYZ_DESCRIPTORS_PATCHES
		};

		MatcherParameters() {
			cameraMatrixMat = cv::Mat::zeros(3, 3, CV_32FC1);
			distortionCoeffsMat = cv::Mat::zeros(1, 5, CV_32FC1);
		}
		;
		MatcherParameters(std::string configFilename,
				std::string cameraConfigFileName) {

			tinyxml2::XMLDocument config;
			std::string filename = "../../resources/" + configFilename;
			config.LoadFile(filename.c_str());
			if (config.ErrorID()) {
				std::cout << "Unable to load Matcher OpenCV config file: "
						<< configFilename << std::endl;
			}
			tinyxml2::XMLElement * params = config.FirstChildElement("Matcher");
			// Matcher
			params->QueryIntAttribute("verbose", &verbose);
			params->QueryIntAttribute("VOVersion", &VOVersion);
			params->QueryIntAttribute("MapMatchingVersion",
                    &MapMatchingVersion);

//			std::cout<<"VOVersion: " << VOVersion << std::endl;
//			std::cout<<"MapMatchingVersion: " << MapMatchingVersion << std::endl;

			// RANSAC
			params->FirstChildElement("RANSAC")->QueryIntAttribute("verbose",
					&RANSACParams.verbose);
			params->FirstChildElement("RANSAC")->QueryIntAttribute(
					"errorVersionVO", &RANSACParams.errorVersionVO);
			params->FirstChildElement("RANSAC")->QueryIntAttribute(
					"errorVersionMap", &RANSACParams.errorVersionMap);
			params->FirstChildElement("RANSAC")->QueryDoubleAttribute(
					"inlierThresholdEuclidean",
					&RANSACParams.inlierThresholdEuclidean);
			params->FirstChildElement("RANSAC")->QueryDoubleAttribute(
					"inlierThresholdReprojection",
					&RANSACParams.inlierThresholdReprojection);
			params->FirstChildElement("RANSAC")->QueryDoubleAttribute(
					"inlierThresholdMahalanobis",
					&RANSACParams.inlierThresholdMahalanobis);
			params->FirstChildElement("RANSAC")->QueryDoubleAttribute(
					"minimalInlierRatioThreshold",
					&RANSACParams.minimalInlierRatioThreshold);
			params->FirstChildElement("RANSAC")->QueryIntAttribute(
					"minimalNumberOfMatches",
					&RANSACParams.minimalNumberOfMatches);
			params->FirstChildElement("RANSAC")->QueryIntAttribute("usedPairs",
					&RANSACParams.usedPairs);



			// general parameters
			params->FirstChildElement("Parameters")->QueryDoubleAttribute(
					"maxAngleBetweenFrames", &maxAngleBetweenFrames);
			// Matcher OpenCV
			OpenCVParams.detector =
					params->FirstChildElement("MatcherOpenCV")->Attribute(
							"detector");
			OpenCVParams.descriptor =
					params->FirstChildElement("MatcherOpenCV")->Attribute(
							"descriptor");

			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"gridRows", &OpenCVParams.gridRows);
			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"gridCols", &OpenCVParams.gridCols);

			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"useInitialFlow", &OpenCVParams.useInitialFlow);
			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"winSize", &OpenCVParams.winSize);

			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"maxLevels", &OpenCVParams.maxLevels);

			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"maxIter", &OpenCVParams.maxIter);
			params->FirstChildElement("MatcherOpenCV")->QueryFloatAttribute(
					"eps", &OpenCVParams.eps);
			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
					"minimalTrackedFeatures",
					&OpenCVParams.minimalTrackedFeatures);
			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
								"maximalTrackedFeatures",
								&OpenCVParams.maximalTrackedFeatures);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
					"minimalReprojDistanceNewTrackingFeatures",
					&OpenCVParams.minimalReprojDistanceNewTrackingFeatures);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
								"minimalEuclidDistanceNewTrackingFeatures",
								&OpenCVParams.minimalEuclidDistanceNewTrackingFeatures);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
					"DBScanEps", &OpenCVParams.DBScanEps);

			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
					"matchingXYZSphereRadius",
					&OpenCVParams.matchingXYZSphereRadius);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
					"matchingXYZacceptRatioOfBestMatch",
					&OpenCVParams.matchingXYZacceptRatioOfBestMatch);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
								"trackingErrorThreshold",
								&OpenCVParams.trackingErrorThreshold);
			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
											"trackingErrorType",
											&OpenCVParams.trackingErrorType);
			params->FirstChildElement("MatcherOpenCV")->QueryDoubleAttribute(
														"trackingMinEigThreshold",
														&OpenCVParams.trackingMinEigThreshold);

			params->FirstChildElement("MatcherOpenCV")->QueryIntAttribute(
																	"removeTooCloseFeatures",
																	&OpenCVParams.removeTooCloseFeatures);


			// Patches params
			params->FirstChildElement("MatchingOnPatches")->QueryIntAttribute(
					"verbose", &PatchesParams.verbose);
			params->FirstChildElement("MatchingOnPatches")->QueryBoolAttribute(
					"warping", &PatchesParams.warping);
			params->FirstChildElement("MatchingOnPatches")->QueryIntAttribute(
					"patchSize", &PatchesParams.patchSize);
			params->FirstChildElement("MatchingOnPatches")->QueryIntAttribute(
					"maxIterationCount", &PatchesParams.maxIter);
			params->FirstChildElement("MatchingOnPatches")->QueryDoubleAttribute(
					"minSqrtError", &PatchesParams.minSqrtIncrement);

			// Camera parameters
			cameraMatrixMat = cv::Mat::zeros(3, 3, CV_32FC1);
			distortionCoeffsMat = cv::Mat::zeros(1, 5, CV_32FC1);

			config.Clear();
			filename = "../../resources/" + cameraConfigFileName;
			config.LoadFile(filename.c_str());
			if (config.ErrorID()) {
				std::cout << "Unable to load camera config file: "
						<< configFilename << std::endl;
			}

			// Get dataset config
			std::string datasetCfgFilename =
					config.FirstChildElement("Model")->Attribute("datasetFile");
			tinyxml2::XMLDocument datasetCfg;
			filename = "../../resources/" + datasetCfgFilename;
			datasetCfg.LoadFile(filename.c_str());
			if (datasetCfg.ErrorID()) {
				std::cout << "Unable to load dataset config file: "
						<< configFilename << std::endl;
			}

			tinyxml2::XMLElement * params2 = datasetCfg.FirstChildElement(
					"Model");

			params2->FirstChildElement("focalLength")->QueryFloatAttribute("fu",
					&cameraMatrixMat.at<float>(0, 0));
			params2->FirstChildElement("focalLength")->QueryFloatAttribute("fv",
					&cameraMatrixMat.at<float>(1, 1));

			params2->FirstChildElement("focalAxis")->QueryFloatAttribute("Cu",
					&cameraMatrixMat.at<float>(0, 2));
			params2->FirstChildElement("focalAxis")->QueryFloatAttribute("Cv",
					&cameraMatrixMat.at<float>(1, 2));
			cameraMatrixMat.at<float>(2, 2) = 1.0f;

			params2->FirstChildElement("rgbDistortion")->QueryFloatAttribute(
					"k1", &distortionCoeffsMat.at<float>(0));
			params2->FirstChildElement("rgbDistortion")->QueryFloatAttribute(
					"k2", &distortionCoeffsMat.at<float>(1));
			params2->FirstChildElement("rgbDistortion")->QueryFloatAttribute(
					"p1", &distortionCoeffsMat.at<float>(2));
			params2->FirstChildElement("rgbDistortion")->QueryFloatAttribute(
					"p2", &distortionCoeffsMat.at<float>(3));
			params2->FirstChildElement("rgbDistortion")->QueryFloatAttribute(
					"k3", &distortionCoeffsMat.at<float>(4));
			config.Clear();

//			std::cout<<"READ CAMERA MODEL:" << std::endl << cameraMatrixMat << std::endl << distortionCoeffsMat << std::endl;

		}
	public:
        int verbose, VOVersion, MapMatchingVersion;
		RANSAC::parameters RANSACParams;
		Matcher::parameters OpenCVParams;
		MatchingOnPatches::parameters PatchesParams;

		cv::Mat cameraMatrixMat;
		cv::Mat distortionCoeffsMat;

		/// max rotation angle between camera frames
        double maxAngleBetweenFrames;
	};

protected:

	/// Matcher name
	const std::string name;

	/// Frame id
	uint_fast32_t frame_id;

	/// Information about previous keypoints + descriptors
	std::vector<cv::KeyPoint> prevKeyPoints;
    std::vector<double> prevDetDists;
	std::vector<cv::Point2f> prevFeaturesUndistorted, prevFeaturesDistorted;
	cv::Mat prevDescriptors;
	std::vector<Eigen::Vector3f> prevFeatures3D;
	cv::Mat prevRgbImage, prevDepthImage;
	double prevDepthImageScale;

	// Time measurement
	TimeMeasurement detectionTimes, trackingTimes, ransacTimes;

	//TODO: TEMPORARILY
public:
	/// Parameters
	MatcherParameters matcherParameters;
protected:

	/// Methods used to visualize results/data
	void showFeatures(cv::Mat rgbImage, std::vector<cv::KeyPoint> features);
	void showMatches(cv::Mat prevRgbImage,
			std::vector<cv::KeyPoint> prevFeatures, cv::Mat rgbImage,
			std::vector<cv::KeyPoint> features,
			std::vector<cv::DMatch> matches);

	/// Detect features
	virtual std::vector<cv::KeyPoint> detectFeatures(cv::Mat rgbImage) = 0;

	/// Describe features
	virtual cv::Mat describeFeatures(cv::Mat rgbImage,
			std::vector<cv::KeyPoint> &features) = 0;

	/// Perform matching
	virtual std::vector<cv::DMatch> performMatching(cv::Mat prevDescriptors,
			cv::Mat descriptors) = 0;

	// Perform tracking
	virtual std::vector<cv::DMatch> performTracking(cv::Mat prevImg,
			cv::Mat img, std::vector<cv::Point2f> &prevFeatures,
			std::vector<cv::Point2f> &features,
			std::vector<cv::KeyPoint>& prevKeyPoints,
			std::vector<cv::KeyPoint>& keyPoints,
            std::vector<double>& prevDetDists,
            std::vector<double>& detDists) = 0;

private:
	// We need to extract values in OpenCV types from classes/structures
	cv::Mat extractMapDescriptors(std::vector<MapFeature> mapFeatures);
	std::vector<Eigen::Vector3f> extractMapFeaturesPositions(
			std::vector<MapFeature> mapFeatures);

	double matchXYZ(std::vector<MapFeature> mapFeatures, int sensorPoseId,
					std::vector<MapFeature> &foundInlierMapFeatures,
					Eigen::Matrix4f &estimatedTransformation,
					cv::Mat currentPoseDescriptors,
					std::vector<Eigen::Vector3f> &currentPoseFeatures3D,
					std::vector<cv::KeyPoint>& currentPoseKeyPoints,
                    std::vector<double>& currentPoseDetDists,
					std::vector<int> frameIds = std::vector<int>());


	std::set<int> removeTooCloseFeatures(std::vector<cv::Point2f>& distortedFeatures2D,
			std::vector<cv::Point2f>& undistortedFeatures2D,
			std::vector<Eigen::Vector3f> &features3D,
			std::vector<cv::KeyPoint>& keyPoints,
            std::vector<double>& detDists,
			std::vector<cv::DMatch> &matches);

	// Method used to combine old tracking features with new features
	void mergeTrackedFeatures(std::vector<cv::Point2f>& undistortedFeatures2D,
			const std::vector<cv::Point2f>& featuresSandBoxUndistorted,
			std::vector<cv::Point2f>& distortedFeatures2D,
			const std::vector<cv::Point2f>& featuresSandBoxDistorted,
			std::vector<Eigen::Vector3f>& features3D,
			const std::vector<Eigen::Vector3f>& features3DSandBox,
			std::vector<cv::KeyPoint>& keyPoints,
			const std::vector<cv::KeyPoint>& keyPointsSandBox,
            std::vector<double>& detDists,
            const std::vector<double>& detDistsSandBox);

	void framesIds2framesIndex(std::vector<MapFeature> featureSet,
			std::vector<int> frameIds,
			std::vector<int> &closestFrameIndex);
};
}
;

#endif // _MATCHER_H_
