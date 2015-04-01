/** @file featuresMap.h
 *
 * implementation - Elevation Map
 *
 */

#ifndef FEATURES_MAP_H_INCLUDED
#define FEATURES_MAP_H_INCLUDED

#include "map.h"
#include "../PoseGraph/graph_g2o.h"
#include <iostream>
#include <memory>
#include <atomic>
#include "../include/Grabber/depthSensorModel.h"

#define FATURES_START_ID 10000

namespace putslam {
/// create a single Map
Map* createFeaturesMap(void);
/// create a single Map - overloaded
Map* createFeaturesMap(std::string configFileGrabber, std::string sensorConfig);
}

using namespace putslam;

class MapModifier{
public:
    /// Features to update
    std::vector<MapFeature> features2update;

    /// Features to remove
    std::vector<int> removeIds;

    /// Features to update
    std::vector<MapFeature> features2add;

    /// Update features?
    inline bool updateFeatures() { return (features2update.size()>0) ?  true : false;};
    /// Remove feaures?
    inline bool removeFeatures() { return (removeIds.size()>0) ?  true : false;};
    /// add features?
    inline bool addFeatures() { return (features2add.size()>0) ?  true : false;};

    /// mutex to lock access
    std::recursive_mutex mtxBuffer;
};

/// Map implementation
class FeaturesMap: public Map {
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
	void addFeatures(const std::vector<RGBDFeature>& features,
			int poseId = -1);

	/// add measurements (features measured from the last camera pose) default: the last sensor pose
	void addMeasurements(const std::vector<MapFeature>& features,
            int poseId = -1);

    /// add measurement between two poses
    void addMeasurement(int poseFrom, int poseTo, Mat34 transformation);

	/// add new pose of the camera, returns id of the new pose
    int addNewPose(const Mat34& cameraPoseChange, float_type timestamp, cv::Mat image = cv::Mat(), cv::Mat depthImage = cv::Mat());

	/// Get all features
	std::vector<MapFeature> getAllFeatures(void);

	/// Get feature position
	Vec3 getFeaturePosition(unsigned int id);

	/// get all visible features
	std::vector<MapFeature> getVisibleFeatures(const Mat34& cameraPose);

    /// find nearest id of the image frame taking into acount the current angle of view and the view from the history
    void findNearestFrame(const std::vector<MapFeature>& features, std::vector<int>& imageIds);

	/// get pose of the sensor (default: last pose)
	Mat34 getSensorPose(int poseId = -1);

	/// getDepthSensorModel
	DepthSensorModel getDepthSensorModel() {
		return sensorModel;
	}

	/// start optimization thread
    void startOptimizationThread(unsigned int iterNo, int verbose = 0, std::string RobustKernelName = "", float_type kernelDelta = 0);

	/// Wait for optimization thread to finish
	void finishOptimization(std::string trajectoryFilename,
			std::string graphFilename);

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

    /// set Robust Kernel
    void setRobustKernel(std::string name, float_type delta);

    /// disable Robust Kernel
    void disableRobustKernel(void);

    /// get n-th image and depth image from the sequence
    void getImages(int poseNo, cv::Mat& image, cv::Mat& depthImage);

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
            model->FirstChildElement( "parameters" )->QueryBoolAttribute("useUncertainty", &useUncertainty);
            model->FirstChildElement( "parameters" )->QueryBoolAttribute("fixVertices", &fixVertices);
            model->FirstChildElement( "parameters" )->QueryIntAttribute("weakFeatureThr", &weakFeatureThr);
            model->FirstChildElement( "parameters" )->QueryFloatAttribute("edges3DPrunningThreshold", &edges3DPrunningThreshold);
			model->FirstChildElement("parameters")->QueryIntAttribute(
					"addFeaturesWhenMapSizeLessThan",
					&addFeaturesWhenMapSizeLessThan);
			model->FirstChildElement("parameters")->QueryIntAttribute(
					"addFeaturesWhenMeasurementSizeLessThan",
					&addFeaturesWhenMeasurementSizeLessThan);
			model->FirstChildElement("parameters")->QueryIntAttribute(
								"maxOnceFeatureAdd",
								&maxOnceFeatureAdd);
			model->FirstChildElement("parameters")->QueryFloatAttribute(
					"minEuclideanDistanceOfFeatures",
					&minEuclideanDistanceOfFeatures);
			model->FirstChildElement("parameters")->QueryFloatAttribute(
								"minImageDistanceOfFeatures",
								&minImageDistanceOfFeatures);
			model->FirstChildElement("parameters")->QueryIntAttribute(
								"addNoFeaturesWhenMapSizeGreaterThan",
								&addNoFeaturesWhenMapSizeGreaterThan);
        }
        public:
            // Use uncertinty model of the camera to determine information matrix in the graph
            bool useUncertainty;// true - use uncertainty model

            // before final optimization remove features with measuremets less than threshold
            int weakFeatureThr;

            /// 3D edges pruning
            float edges3DPrunningThreshold;

            // fix all optimized vertices after optimization
            bool fixVertices;

            // We perform adding to map if visible map is too small
            int addFeaturesWhenMapSizeLessThan;

            // We perform adding when we have less feature observations than
            int addFeaturesWhenMeasurementSizeLessThan;

            // We maximally can add this number of features from current frame
            int maxOnceFeatureAdd;

            // We do not add features closer to already existing ones than
            float minEuclideanDistanceOfFeatures;
            float minImageDistanceOfFeatures;

            // If we observe many features, we do not add new
            int addNoFeaturesWhenMapSizeGreaterThan;
    };

private:
    ///Configuration of the module
    Config config;

	///camera trajectory
    std::vector<VertexSE3> camTrajectory;

    ///odometry -- transformations beetween camera poses
    std::vector<Mat34> odoMeasurements;

    /// RGB camera images -- sequence
    std::deque<cv::Mat> imageSeq;

    /// Depth camera images -- sequence
    std::deque<cv::Mat> depthSeq;

    /// mutex for camera trajectory
    std::mutex mtxCamTraj;

	///Pose graph
	Graph * poseGraph;

	/// Depth sensor model
	DepthSensorModel sensorModel;

	/// Optimization thread
	std::unique_ptr<std::thread> optimizationThr;

	/// optimization flag
	std::atomic<bool> continueOpt;

	/// Number of features
	unsigned int featureIdNo;

	/// boolean value informing if the features had been added to the map
	bool emptyMap;

    ///Set of features (map for the front-end thread)
    std::vector<MapFeature> featuresMapFrontend;

    /// mutex for critical section - map frontend
    std::recursive_mutex mtxMapFrontend;

    ///Set of features (map for the visualization thread)
    std::vector<MapFeature> featuresMapVisualization;

    /// mutex for critical section - map visualization
    std::recursive_mutex mtxMapVisualization;

    ///Set of features (map for the map management thread)
    std::vector<MapFeature> featuresMapManagement;

    /// mutex for critical section - map management
    std::recursive_mutex mtxMapManagemet;

    /// Map frontend -- buffer
    MapModifier bufferMapFrontend;

    /// Map visualization -- buffer
    MapModifier bufferMapVisualization;

    /// Map management -- buffer
    MapModifier bufferMapManagement;

    /// Last optimized pose
    int lastOptimizedPose;

	/// optimization thread
    void optimize(unsigned int iterNo, int verbose, std::string RobustKernelName = "", float_type kernelDelta = 0);

    /// Update map
    void updateMap(MapModifier& modifier, std::vector<MapFeature>& featuresMap, std::recursive_mutex& mutex);

    /// Update feature
    void updateFeature(std::vector<MapFeature>& featuresMap, MapFeature& newFeature);

    /// Update camera trajectory
    void updateCamTrajectory(std::vector<VertexSE3>& poses2update);

    /// Update pose
    void updatePose(VertexSE3& newPose);

    /// plot all features
    void plotFeatures(std::string filename);
};

#endif // FEATURES_MAP_H_INCLUDED
