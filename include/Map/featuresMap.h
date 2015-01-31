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
    Map* createFeaturesMap(std::string sensorConfig);
};

using namespace putslam;

/// Map implementation
class FeaturesMap : public Map {
    public:
        /// Pointer
        typedef std::unique_ptr<FeaturesMap> Ptr;

        /// Construction
        FeaturesMap(void);

        /// Construction
        FeaturesMap(std::string sensorConfig);

        /// Destruction
        ~FeaturesMap(void);

        /// Name of the map
        const std::string& getName() const;

        /// Add NEW features and a NEW camera pose (initial guess) to the map
        /// Position of features in relation to camera pose
        void addFeatures(const std::vector<RGBDFeature>& features, const Mat34& cameraPose);

        /// Get all features
        std::vector<MapFeature>& getAllFeatures(void);

        /// Get feature position
        Vec3 getFeaturePosition(unsigned int id);

        /// get all visible features
        std::vector<MapFeature>& getVisibleFeatures(const Mat34& cameraPose);

        /// get current pose of the sensor
        Mat34 getCurrentPose(void);

        /// start optimization thread
        void startOptimizationThread(unsigned int iterNo);

        /// Wait for optimization thread to finish
        void finishOptimization();

    private:
        ///Set of features (map)
        std::vector<MapFeature> featuresSet;

        ///camera trajectory
        std::vector<Mat34> camTrajectory;

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

        /// optimization thread
        void optimize(unsigned int iterNo);
};

#endif // FEATURES_MAP_H_INCLUDED
