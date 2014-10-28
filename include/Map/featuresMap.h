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

namespace putslam {
    /// create a single Map
    Map* createFeaturesMap(void);
};

using namespace putslam;

/// Map implementation
class FeaturesMap : public Map {
    public:
        /// Pointer
        typedef std::unique_ptr<FeaturesMap> Ptr;

        /// Construction
        FeaturesMap(void);

        /// Name of the map
        const std::string& getName() const;

        /// Add new features and camera pose (initial guess) to the map
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
        void startOptimizationThread();

    private:
        ///Set of features (map)
        std::vector<MapFeature> features;

        ///Pose graph
        Graph * poseGraph;
};

#endif // FEATURES_MAP_H_INCLUDED
