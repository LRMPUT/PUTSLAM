/** @file featuresMap.h
 *
 * implementation - Elevation Map
 * \author Dominik Belter
 */

#ifndef FEATURES_MAP_H_INCLUDED
#define FEATURES_MAP_H_INCLUDED

#include "map.h"
#include "../PoseGraph/graph_g2o.h"
#include "../PoseGraph/weightedGraph.h"
#include "../include/Utilities/observer.h"
#include "../include/Utilities/stopwatch.h"
#include <memory>
#include <atomic>
#include "../include/Grabber/depthSensorModel.h"
#include "LoopClosure/loopClosureLocal.h"
#include <iostream>
#include <deque>
#include <queue>

#define FEATURES_START_ID 10000

namespace putslam {
/// create a single Map
Map* createFeaturesMap(void);
/// create a single Map - overloaded
Map* createFeaturesMap(std::string configFileGrabber, std::string sensorConfig);
}

using namespace putslam;

/// Map implementation
class FeaturesMap: public Map, public Subject {
public:
	/// Pointer
	typedef std::unique_ptr<FeaturesMap> Ptr;

	/// Construction
	FeaturesMap(void);

	/// Construction
	FeaturesMap(std::string configFileGrabber, std::string sensorConfig);

	/// Destruction
	~FeaturesMap(void);

	/// Name of the map
	const std::string& getName() const;

	/// Add NEW features and a NEW camera pose (initial guess) to the map
	/// Position of features in relation to camera pose, default: the last sensor pose
    void addFeatures(const std::vector<RGBDFeature>& features, int poseId = -1);

	/// add measurements (features measured from the last camera pose) default: the last sensor pose
    void addMeasurements(const std::vector<MapFeature>& features, int poseId = -1);

    /// add measurement between two poses
    void addMeasurement(int poseFrom, int poseTo, Mat34 transformation);

	/// add new pose of the camera, returns id of the new pose
    int addNewPose(const Mat34& cameraPoseChange, float_type timestamp, cv::Mat image = cv::Mat(), cv::Mat depthImage = cv::Mat());

	/// Get all features
	std::vector<MapFeature> getAllFeatures(void);

	/// Get feature position
    Vec3 getFeaturePosition(unsigned int id) const;

	/// get all visible features
	std::vector<MapFeature> getVisibleFeatures(const Mat34& cameraPose);

    /// get all covisible features using covisibility graph
    std::vector<MapFeature> getCovisibleFeatures(void);

    /// get all visible features and reduce results
    std::vector<MapFeature> getVisibleFeatures(const Mat34& cameraPose, int graphDepthThreshold, float_type distanceThreshold);

    /// removes features which are too far from current camera pose (distant in graph)
    void removeDistantFeatures(std::vector<MapFeature>& mapFeatures, int graphDepthThreshold = 0, float_type distanceThreshold = 0);

    /// find nearest id of the image frame taking into acount the current angle of view and the view from the history
    void findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds, std::vector<float_type>& angles, float_type maxAngle = 3.14);

	/// get pose of the sensor (default: last pose)
    Mat34 getSensorPose(int poseId = -1) const;

	/// get size of poses
	int getPoseCounter();

	/// getDepthSensorModel
	DepthSensorModel getDepthSensorModel() {
		return sensorModel;
	}

	/// start optimization thread
    void startOptimizationThread(unsigned int iterNo, int verbose = 0, std::string RobustKernelName = "", float_type kernelDelta = 0);

    /// start map management thread
    void startMapManagerThread(int verbose = 0);

    /// start loop closure thread
    void startLoopClosureThread(int verbose, Matcher* matcher);

	/// Wait for optimization thread to finish
	void finishOptimization(std::string trajectoryFilename,
			std::string graphFilename);

    /// Export graph and trajectory
    void exportOutput(std::string trajectoryFilename, std::string graphFilename);

    /// Wait for map management thread to finish
    void finishManagementThr(void);

    /// Wait for loop closure thread to finish
    void finishLoopClosureThr(void);

    /// Save map to file
    void save2file(std::string mapFilename, std::string graphFilename);

	/// Get some parameters
	int getAddFeaturesWhenMapSizeLessThan() {
		return config.addFeaturesWhenMapSizeLessThan;
	}
	int getAddFeaturesWhenMeasurementSizeLessThan() {
		return config.addFeaturesWhenMeasurementSizeLessThan;
	}
	int getMaxOnceFeatureAdd() {
		return config.maxOnceFeatureAdd;
	}
	float getMinEuclideanDistanceOfFeatures() {
		return config.minEuclideanDistanceOfFeatures;
	}
	float getMinImageDistanceOfFeatures() {
		return config.minImageDistanceOfFeatures;
	}
	int getAddNoFeaturesWhenMapSizeGreaterThan() {
		return config.addNoFeaturesWhenMapSizeGreaterThan;
	}
	int getMinMeasurementsToAddPoseToFeatureEdge() {
		return config.minMeasurementsToAddPoseToFeatureEdge;
	}
	bool getAddPoseToPoseEdges() {
		return config.addPoseToPoseEdges;
	}

    /// set Robust Kernel
    void setRobustKernel(std::string name, float_type delta);

    /// disable Robust Kernel
    void disableRobustKernel(void);

    /// get n-th image and depth image from the sequence
    void getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage);

    /// Update pose
    void updatePose(VertexSE3& newPose, bool updateGraph = false);

    /// get uncertainty of the pose
    Mat66 getPoseUncertainty(unsigned int id) const;

    /// get uncertainty of the feature
    Mat33 getFeatureUncertainty(unsigned int id) const;

    /// set drawing options
    void setDrawOptions(bool _draw);

    /// use uncertainty
    bool useUncertainty(void);

    /// store camera frames
    void setStoreImages(bool storeImages);

    /// return the size of the map
    int getNumberOfFeatures();

    class Config{
      public:
        Config() :
            useUncertainty(true){
        }
        Config(std::string configFilename){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load Map config file.\n";
            tinyxml2::XMLElement * model = config.FirstChildElement( "MapConfig" );
            model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
            model->FirstChildElement( "parameters" )->QueryBoolAttribute("useUncertainty", &useUncertainty);
            model->FirstChildElement( "parameters" )->QueryIntAttribute("uncertaintyModel", &uncertaintyModel);
            model->FirstChildElement( "parameters" )->QueryBoolAttribute("fixVertices", &fixVertices);
            model->FirstChildElement( "parameters" )->QueryBoolAttribute("addPoseToPoseEdges", &addPoseToPoseEdges);
            model->FirstChildElement( "parameters" )->QueryIntAttribute("minMeasurementsToAddPoseToFeatureEdge", &minMeasurementsToAddPoseToFeatureEdge);
            model->FirstChildElement( "parameters" )->QueryIntAttribute("weakFeatureThr", &weakFeatureThr);
            model->FirstChildElement( "parameters" )->QueryDoubleAttribute("edges3DPrunningThreshold", &edges3DPrunningThreshold);
			model->FirstChildElement("parameters")->QueryIntAttribute(
					"addFeaturesWhenMapSizeLessThan",
					&addFeaturesWhenMapSizeLessThan);
			model->FirstChildElement("parameters")->QueryIntAttribute(
					"addFeaturesWhenMeasurementSizeLessThan",
					&addFeaturesWhenMeasurementSizeLessThan);
			model->FirstChildElement("parameters")->QueryIntAttribute(
								"maxOnceFeatureAdd",
								&maxOnceFeatureAdd);
            model->FirstChildElement("parameters")->QueryDoubleAttribute(
					"minEuclideanDistanceOfFeatures",
					&minEuclideanDistanceOfFeatures);
            model->FirstChildElement("parameters")->QueryDoubleAttribute(
								"minImageDistanceOfFeatures",
                                &minImageDistanceOfFeatures);
            model->FirstChildElement("parameters")->QueryIntAttribute(
                                "addNoFeaturesWhenMapSizeGreaterThan",
                                &addNoFeaturesWhenMapSizeGreaterThan);
			model->FirstChildElement("parameters")->QueryIntAttribute(
											"optimizationErrorType",
											&optimizationErrorType);

            model->FirstChildElement("mapCompression")->QueryBoolAttribute("compressMap", &compressMap);
            model->FirstChildElement("mapCompression")->QueryDoubleAttribute("covisibilityKeyframes", &covisibilityKeyframes);
            model->FirstChildElement("mapCompression")->QueryDoubleAttribute("marginalizationThr", &marginalizationThr);
            model->FirstChildElement("mapCompression")->QueryIntAttribute("minFramesNo", &minFramesNo);
            model->FirstChildElement("mapCompression")->QueryIntAttribute("maxFramesNo", &maxFramesNo);

            model->FirstChildElement("EuclideanCriterion")->QueryBoolAttribute("useEuclideanCrit", &useEuclideanCrit);
            model->FirstChildElement("EuclideanCriterion")->QueryDoubleAttribute("imagePlaneDistance", &imagePlaneDistance);
            model->FirstChildElement("EuclideanCriterion")->QueryDoubleAttribute("depthDist", &depthDist);
            model->FirstChildElement("EuclideanCriterion")->QueryDoubleAttribute("maxAngle", &maxAngle);
            model->FirstChildElement("EuclideanCriterion")->QueryDoubleAttribute("maxRadius", &maxRadius);

            model->FirstChildElement( "mapOutput" )->QueryBoolAttribute("exportMap", &exportMap);
            filenameMap = model->FirstChildElement( "mapOutput" )->Attribute("filenameMap");
            filenameData = model->FirstChildElement( "mapOutput" )->Attribute("filenameData");
            model->FirstChildElement( "mapManager" )->QueryDoubleAttribute("distThreshold", &distThreshold);
            model->FirstChildElement( "featuresDistribution" )->QueryBoolAttribute("exportDistribution", &exportDistribution);
            model->FirstChildElement( "featuresDistribution" )->QueryUnsignedAttribute("frameNo", &frameNo);
            filenameFeatDistr = model->FirstChildElement( "featuresDistribution" )->Attribute("filenameFeatDistr");
            model->FirstChildElement( "visualization" )->QueryIntAttribute("frameNo2updatePointCloud", &frameNo2updatePointCloud);

            model->FirstChildElement( "loopClosure" )->QueryIntAttribute("searchPairsTypeLC", &searchPairsTypeLC);
            model->FirstChildElement( "loopClosure" )->QueryIntAttribute("waitUntilFinishedLC", &waitUntilFinishedLC);
            model->FirstChildElement( "loopClosure" )->QueryIntAttribute("minNumberOfFeaturesLC", &minNumberOfFeaturesLC);
            model->FirstChildElement( "loopClosure" )->QueryIntAttribute("measurementTypeLC", &measurementTypeLC);
            model->FirstChildElement( "loopClosure" )->QueryDoubleAttribute("matchingRatioThresholdLC", &matchingRatioThresholdLC);
            model->FirstChildElement( "loopClosure" )->QueryIntAttribute("typeLC", &typeLC);
            configFilenameLC = model->FirstChildElement( "loopClosure" )->Attribute("configFilenameLC");



            visualize = false;

        }
        public:
            //verbose
            int verbose;

            // Use uncertinty model of the camera to determine information matrix in the graph
            bool useUncertainty;// true - use uncertainty model

            // uncetainty model
            int uncertaintyModel;

            // before final optimization remove features with measuremets less than threshold
            int weakFeatureThr;

            /// 3D edges pruning
            double edges3DPrunningThreshold;

            /// Get features from map using Euclidean criterion
            bool useEuclideanCrit;

            /// Euclidean distance on the image plane
            double imagePlaneDistance;

            /// Distance along the camera axis
            double depthDist;

            /// max angle
            double maxAngle;

            /// max radius (do not search for neighbour if radius bigger than thr)
            double maxRadius;

            /// if covisibility smaller than 'marginCovisibThr' then marginalize graph
            double marginalizationThr;

            /// min Frames No for marginalization
            int minFramesNo;

            /// if frames no bigger than maxFramesNo then marginalize
            int maxFramesNo;

            // fix all optimized vertices after optimization
            bool fixVertices;

            // true/false whether add pose-pose constrains from VO
            bool addPoseToPoseEdges;

            // minimal number of consistent pose-(feature in map) observations to add those links
            int minMeasurementsToAddPoseToFeatureEdge;

            // We perform adding to map if visible map is too small
            int addFeaturesWhenMapSizeLessThan;

            // We perform adding when we have less feature observations than
            int addFeaturesWhenMeasurementSizeLessThan;

            // We maximally can add this number of features from current frame
            int maxOnceFeatureAdd;

            // We do not add features closer to already existing ones than
            double minEuclideanDistanceOfFeatures;
            double minImageDistanceOfFeatures;

            // If we observe many features, we do not add new
            int addNoFeaturesWhenMapSizeGreaterThan;

            /// MapManagement: distance threshold
            float_type distThreshold;

            /// export map to files
            bool exportMap;

            /// m-file ploting all features
            std::string filenameMap;

            /// m-file computing map properties
            std::string filenameData;

            /// draw each detected feature in the common coordinate system
            bool exportDistribution;

            /// m-file ploting features distribution
            std::string filenameFeatDistr;

            /// common frame no for features distribution
            unsigned int frameNo;

            /// Update point cloud visualizer ever n-th frame
            int frameNo2updatePointCloud;

            /// use visualizer
            bool visualize;

            /// wait [s] until break LC thread
            int waitUntilFinishedLC;

            /// minimal number of features to check LC
            int minNumberOfFeaturesLC;

            /// type of measurement added to the graph
            int measurementTypeLC;

            /// LoopClosure: matchingRatioThreshold
            double matchingRatioThresholdLC;

            /// type LC
            int typeLC;

            /// method which pairs poses
            int searchPairsTypeLC;

            /// config filename LC
            std::string configFilenameLC;

            /// store images
            bool keepCameraFrames;

            /// type of error used in optimization
            int optimizationErrorType;

            /// use map compression
            bool compressMap;

            /// add keyframe when covisibility smaler than
            double covisibilityKeyframes;

            enum OptimizationErrorType {
            	EUCLIDEAN,
				REPROJECTION
            };
    };

private:
    ///Configuration of the module
    Config config;

	///camera trajectory
    std::vector<VertexSE3> camTrajectory;

    /// last keyframe id
    int lastKeyframeId;

    /// active keyframesNo
    int activeKeyframesNo;

    ///lastFullyMarginalizedFrame
    int lastFullyMarginalizedFrame;

    /// frames (range) for marginalization
    std::pair<int,int> frames2marginalize;

    ///odometry -- transformations beetween camera poses
    std::vector<Mat34> odoMeasurements;

    /// RGB camera images -- sequence
    std::map<int,cv::Mat> imageSeq;

    /// Depth camera images -- sequence
    std::map<int,cv::Mat> depthSeq;

    /// mutex for camera trajectory
    mutable std::mutex mtxCamTraj;

	///Pose graph
	Graph * poseGraph;

    /// Covisibility graph
    WeightedGraph covisibilityGraph;

	/// Depth sensor model
	DepthSensorModel sensorModel;

	/// Optimization thread
	std::unique_ptr<std::thread> optimizationThr;

    /// <timestamp, opt time>
    std::list<std::pair<double, double>> optimizationTime;

    /// Optimization thread
    std::unique_ptr<std::thread> managementThr;

    /// Loop closure thread
    std::unique_ptr<std::thread> loopClosureThr;

	/// optimization flag
	std::atomic<bool> continueOpt;

    /// map management thread flag
    std::atomic<bool> continueManagement;

    /// loop closure thread flag
    std::atomic<bool> continueLoopClosure;

    ///camera trajectory
    std::vector<VertexSE3> camTrajectoryLC;

    /// mutex for camera trajectory
    std::mutex mtxCamTrajLC;

	/// Number of features
	unsigned int featureIdNo;

	/// boolean value informing if the features had been added to the map
	bool emptyMap;

    ///Set of features (map for the front-end thread)
    std::map<int,MapFeature> featuresMapFrontend;

    /// mutex for critical section - map frontend
    mutable std::recursive_mutex mtxMapFrontend;

    ///Set of features (map for the map management thread)
    std::map<int,MapFeature> featuresMapManagement;

    /// mutex for critical section - map management
    std::recursive_mutex mtxMapManagement;

    ///Set of features (map for the loop closure thread)
    std::map<int,MapFeature> featuresMapLoopClosure;

    /// mutex for critical section - bufferTrajectoryLC
    std::recursive_mutex mtxMapLoopClosure;

    /// Map frontend -- buffer
    MapModifier bufferMapFrontend;

    /// Map visualization -- buffer
    MapModifier bufferMapVisualization;

    /// Map management -- buffer
    MapModifier bufferMapManagement;

    /// Loop closure -- buffer
    MapModifier bufferMapLoopClosure;

    /// Last optimized pose
    int lastOptimizedPose;

    /// loop closure priority queue management
    LoopClosure* localLC;

    /// The log of matchin ratios found in loop closure
    std::vector<double> loopClosureMatchingRatiosLog;
    std::vector<std::pair<int, int>> loopClosureAnalyzedPairsLog;

    // Was Loop Closure successful?
    bool loopClosureSuccess;

    /// get loopClosureMatchingRatiosLog
    std::vector<double> getLoopClosureMatchingRatiosLog();
    std::vector<std::pair<int, int>> getLoopClosureAnalyzedPairsLog();

    // Was loop closure successful
    bool getAndResetLoopClosureSuccesful();

    /// optimization method
    void optimize(unsigned int iterNo, int verbose, std::string RobustKernelName = "", float_type kernelDelta = 0);

    /// map management method
    void manage(int verbose);

    /// geometric loop closure method
    void loopClosure(int verbose, Matcher* matcher);

    /// Update map
    void updateMap(MapModifier& modifier, std::map<int,MapFeature>& featuresMap, std::recursive_mutex& mutex);

    /// Update feature
    void updateFeature(std::map<int,MapFeature>& featuresMap, MapFeature& newFeature);

    /// Update camera trajectory
    void updateCamTrajectory(std::vector<VertexSE3>& poses2update);

    /// plot all features
    void plotFeatures(std::string filenamePlot, std::string filenameData);

    /// plot all features on the i-th image
    void plotFeaturesOnImage(std::string filename, unsigned int frameId);

    /// computes std and mean from float vector
    void computeMeanStd(const std::vector<float_type>& v, float_type& mean, float_type& std, float_type& max);

    /// marginalize measurements between frames
    void marginalizeMeasurements(int frameBegin, int frameEnd);

    /// compute covisibility between current frame and previous keyframes, returns max covisibility
    double computeCovisibility(const std::vector<MapFeature>& features, std::vector<std::pair<int,double>>& covisibilityKeyframes) const;

    /// Restore camera frames (previously marginalized)
    void restoreFrames(void);

    /// get nerby camera poses using Euclidean criteria
    std::set<int> getNearbyPoses(int frameId, int startFrameId);

    /// fixMeasurementsFromPose
    void fixMeasurementsFromPose(int frameId);

    /// save optimization time
    void saveOptimizationTime(std::list<std::pair<double,double>>& optimizationTime, std::string filename);
};

#endif // FEATURES_MAP_H_INCLUDED
