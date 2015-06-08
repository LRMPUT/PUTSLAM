#ifndef _PUTSLAM_
#define _PUTSLAM_

#include <thread>
#include <opencv/highgui.h>
#include <cmath>
#include <ctime>
#include <ratio>
#include <chrono>
#include <fstream>

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

using namespace std;
using namespace putslam;

class PUTSLAM {
	Map* map;
	Grabber* grabber;
	Matcher * matcher;

	ofstream trajectoryFreiburgStream;
	ofstream trajectoryVOMapStream;

    Eigen::Matrix4f robotPose;
    Eigen::Matrix4f VoMapPose;

public:

	PUTSLAM() {

		loadConfigs();

		// Reading robot starting pose
		robotPose = grabber->getStartingSensorPose();
		VoMapPose = robotPose;

		// File to save trajectory
		trajectoryFreiburgStream.open("VO_trajectory.res");
		trajectoryVOMapStream.open("VOMap_trajectory.res");

	}

	void startProcessing();

    ///Attach visualizer
    void attachVisualizer(QGLVisualizer * visualizer);

    /// get depth sensor model
    inline DepthSensorModel getDepthSensorModel(){
        return map->getDepthSensorModel();
    }

private:
	enum MAPMANAGMENTTHREAD { MAPTHREAD_OFF, MAPTHREAD_ON };
	enum OPTIMIZATIONTHREAD { OPTTHREAD_OFF, OPTTHREAD_ATEND, OPTTHREAD_ON, OPTTHREAD_ON_ROBUSTKERNEL };
	int verbose, mapManagmentThreadVersion, optimizationThreadVersion;



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
};

#endif // _PUTSLAM_
