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

using namespace std;
using namespace putslam;

class PUTSLAM {
	Map* map;
	Grabber* grabber;
	Matcher * matcher, *loopClosureMatcher;

	ofstream trajectoryFreiburgStream;
	ofstream trajectoryVOMapStream;
	ofstream trajectoryMotionModelStream;

    Eigen::Matrix4f robotPose;
    Eigen::Matrix4f VoMapPose;
    Eigen::Matrix4f motionModelPose;

    /// visualization options
    bool visualize;
    bool drawImages;


    TimeMeasurement timeMeasurement;

public:

	PUTSLAM() {

		loadConfigs();

		// Reading robot starting pose
		robotPose = grabber->getStartingSensorPose();
		VoMapPose = robotPose;

		// File to save trajectory
		trajectoryFreiburgStream.open("VO_trajectory.res");
		trajectoryVOMapStream.open("VOMap_trajectory.res");
		trajectoryMotionModelStream.open("MotionModel_trajectory.res");
        drawImages = false;
        visualize = false;
        map->setDrawOptions(false);
	}

	void startProcessing();

    ///Attach visualizer
    void attachVisualizer(QGLVisualizer * visualizer);

    /// get depth sensor model
    inline DepthSensorModel getDepthSensorModel(){
        return map->getDepthSensorModel();
    }
    //play trajectory
    void startPlaying(std::string trajectoryFilename, int delayPlay);

    /// set drawing options
    void setDrawOptions(bool _draw, bool _drawImages);

private:
    enum LOOPCLOSURETHREAD { LCTHREAD_OFF, LCTHREAD_ON };
	enum MAPMANAGMENTTHREAD { MAPTHREAD_OFF, MAPTHREAD_ON };
	enum OPTIMIZATIONTHREAD { OPTTHREAD_OFF, OPTTHREAD_ATEND, OPTTHREAD_ON, OPTTHREAD_ON_ROBUSTKERNEL };
    int verbose, onlyVO, mapManagmentThreadVersion, optimizationThreadVersion, loopClosureThreadVersion, octomap, octomapCloudStepSize, octomapOffline;
	double octomapResolution;
    bool keepCameraFrames;
	std::string octomapFileToSave;

	// Octomap pointer
	std::unique_ptr<octomap::ColorOcTree> octomapTree;

	// Save some statistics to analyze
	std::vector<int> measurementToMapSizeLog;
	std::vector<double> VORansacInlierRatioLog;
	std::vector<double> MapMatchingRansacInlierRatioLog;
	std::vector<std::pair<double, double>> patchesErrorLog;
	void saveLogs();
    void saveFPS(float_type fps);

	// Evaluate results
	void evaluateResults(std::string basePath, std::string datasetName);

	void loadConfigs();

	// TODO: Make it nicer
	void saveTrajectoryFreiburgFormat(Eigen::Matrix4f transformation,
			std::ofstream & estTrajectory, double timestamp);
	void saveFeaturesToFile(Matcher::featureSet features, double timestamp);
	void saveFeaturesToFile(Matcher::featureSet features,
			std::vector<cv::DMatch> inlierMatches, double timestamp);
	void moveMapFeaturesToLocalCordinateSystem(const Mat34& cameraPose,
			std::vector<MapFeature>& mapFeatures);
	int chooseFeaturesToAddToMap(const Matcher::featureSet& features,
			int addedCounter, int maxOnceFeatureAdd,
			const std::vector<MapFeature>& mapFeatures,
			float minEuclideanDistanceOfFeatures,
			float minImageDistanceOfFeatures, int cameraPoseId,
			std::vector<RGBDFeature>& mapFeaturesToAdd);

	// Show features from map
	void showMapFeatures(cv::Mat rgbImage,std::vector<MapFeature> mapFeatures);

	// Remove features that we do not have a good observation angle
	void removeMapFeaturesWithoutGoodObservationAngle(
			std::vector<MapFeature> &mapFeatures, std::vector<int> &frameIds,
			std::vector<float_type> &angles);

	void createAndSaveOctomap(double depthImageScale);
	void createAndSaveOctomapOffline( double depthImageScale);
};

#endif // _PUTSLAM_
