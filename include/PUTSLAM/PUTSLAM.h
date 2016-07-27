#ifndef _PUTSLAM_
#define _PUTSLAM_

#include <thread>
#include <opencv/highgui.h>
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>
#include <fstream>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "../include/Defs/putslam_defs.h"
#include "PoseGraph/graph_g2o.h"
#include "PoseGraph/global_graph.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "../include/Grabber/fileGrabber.h"
#include "../include/Grabber/fileGrabber.h"
#include "../include/Grabber/kinectGrabber.h"
#include "../include/Grabber/xtionGrabber.h"
#include "../include/Matcher/matcherOpenCV.h"
#include "../include/Map/featuresMap.h"
#include "../include/RGBD/RGBD.h"
#include "../include/Visualizer/Qvisualizer.h"

#include "TimeMeasurement.h"

#ifdef BUILD_WITH_ROS
#include <nav_msgs/Path.h>  //////////////////////////////////////////////////ROS
#include <tf/tf.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "../include/Grabber/ROSGrabber.h"
#endif

using namespace std;
using namespace putslam;

/*
 * Class used to process RGBD images to retrieve trajectory!
 */
class PUTSLAM {
	Map* map;
	Grabber* grabber;
	Matcher * matcher, *loopClosureMatcher;

	ofstream trajectoryFreiburgStream;
	ofstream trajectoryVOMapStream;
	ofstream trajectoryMotionModelStream;

	Eigen::Matrix4f VOPoseEstimate;
	Eigen::Matrix4f VoMapPose;

	/// visualization options
	bool visualize;bool drawImages;
	TimeMeasurement timeMeasurement;

	// TODO: Some parameters - it is NASTY!
	int addFeaturesWhenMapSizeLessThan, addFeaturesWhenMeasurementSizeLessThan,
			maxOnceFeatureAdd;
	double minEuclideanDistanceOfFeatures, minImageDistanceOfFeatures;
    int addNoFeaturesWhenMapSizeGreaterThan,
            minMeasurementsToAddPoseToFeatureEdge;bool addPoseToPoseEdges;
	double depthImageScale, getVisibleFeaturesGraphMaxDepth,
			getVisibleFeatureDistanceThreshold;

	int verbose, onlyVO, mapManagmentThreadVersion, optimizationThreadVersion,
			loopClosureThreadVersion, octomap, octomapCloudStepSize,
			octomapOffline;
	double octomapResolution;bool keepCameraFrames;
	std::string octomapFileToSave;

	// Octomap pointer
	std::unique_ptr<octomap::ColorOcTree> octomapTree;

	// Save some statistics to analyze
	std::vector<int> measurementToMapSizeLog, VOFeaturesSizeLog;
	std::vector<double> VORansacInlierRatioLog;
	std::vector<double> MapMatchingRansacInlierRatioLog, mapSize;
	std::vector<std::pair<double, double>> patchesErrorLogs;

	enum LOOPCLOSURETHREAD {
		LCTHREAD_OFF, LCTHREAD_ON
	};
	enum MAPMANAGMENTTHREAD {
		MAPTHREAD_OFF, MAPTHREAD_ON
	};
	enum OPTIMIZATIONTHREAD {
		OPTTHREAD_OFF, OPTTHREAD_ATEND, OPTTHREAD_ON, OPTTHREAD_ON_ROBUSTKERNEL
	};

public:

	PUTSLAM();

	void startProcessing();

	///Attach visualizer
	void attachVisualizer(QGLVisualizer * visualizer);

	/// get depth sensor model
	inline DepthSensorModel getDepthSensorModel() {
		return map->getDepthSensorModel();
	}

	//play trajectory
	void startPlaying(std::string trajectoryFilename, int delayPlay);

	/// set drawing options
	void setDrawOptions(bool _draw, bool _drawImages);
	
#ifdef BUILD_WITH_ROS	
	/////////////////////////////////////////////////////////////////////////////ROS
	void setWorkWithROS();
	void initROSpublishers();
#endif

private:
	// At beggining
	void loadConfigs();
	void readingSomeParameters();
	void initialization();

	// Processing
	void processFirstFrame(SensorFrame &currentSensorFrame, int &cameraPoseId);

	Eigen::Matrix4f runVO(SensorFrame &currentSensorFrame,
			std::vector<cv::DMatch> &inlierMatches);
	void addPoseToMap(SensorFrame &currentSensorFrame,
			Eigen::Matrix4f &poseIncrement, int &cameraPoseId);
	Mat34 getMapPoseEstimate();
	Eigen::Matrix4f getPoseIncrementFromMap(int frameCounter);

	void moveMapFeaturesToLocalCordinateSystem(const Mat34& cameraPose,
			std::vector<MapFeature>& mapFeatures);
	int chooseFeaturesToAddToMap(const Matcher::featureSet& features,
			int addedCounter, int maxOnceFeatureAdd,
			const std::vector<MapFeature>& mapFeatures,
			float minEuclideanDistanceOfFeatures,
			float minImageDistanceOfFeatures, int cameraPoseId,
			std::vector<RGBDFeature>& mapFeaturesToAdd);
	// Remove features that we do not have a good observation angle
	void removeMapFeaturesWithoutGoodObservationAngle(
			std::vector<MapFeature> &mapFeatures, std::vector<int> &frameIds,
            std::vector<double> &angles);
	std::vector<MapFeature> getAndFilterFeaturesFromMap(
			SensorFrame &currentSensorFrame, Mat34 cameraPose,
            std::vector<int> &frameIds, std::vector<double> &angles);

	bool removeCloseFeatures(std::vector<RGBDFeature> &existingFeatures,
			Eigen::Vector3f feature3D, cv::Point2f feature2D, double minEuclideanDistanceOfFeatures, double minImageDistanceOfFeatures);
	bool removeCloseFeatures(const std::vector<MapFeature> &existingFeatures,
			Eigen::Vector3f feature3D, cv::Point2f feature2D, double minEuclideanDistanceOfFeatures, double minImageDistanceOfFeatures);

	// At finish
	void saveLogs();
    void saveFPS(double fps);
	void evaluateResults(std::string basePath, std::string datasetName);
	void saveStatistics();

	void saveTrajectoryFreiburgFormat(Eigen::Matrix4f transformation,
			std::ofstream & estTrajectory, double timestamp);
	void saveFeaturesToFile(Matcher::featureSet features, double timestamp);
	void saveFeaturesToFile(Matcher::featureSet features,
			std::vector<cv::DMatch> inlierMatches, double timestamp);
	void showMapFeatures(cv::Mat rgbImage, std::vector<MapFeature> mapFeatures, int wait, string windowName="Map features");
	void createAndSaveOctomap(double depthImageScale);
	void createAndSaveOctomapOffline(double depthImageScale);
	
#ifdef BUILD_WITH_ROS
	//////////////////////////////////////////////////////////////////////////ROS
	ros::NodeHandle nh;
	ros::Publisher cameraOdometryPublisher;
	ros::Publisher cameraPointCloudPublisher;
	ros::Time current_time, last_time; 
	bool workWithROS;
	
	void publishPoseROS(int cameraPoseId);
	void publishPointCloudROS(int cameraPoseId, const SensorFrame &currentSensorFrame);
#endif
};

#endif // _PUTSLAM_
