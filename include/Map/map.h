/** @file map.h
 *
 * Environment Map interface
 *
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Map interface
    class Map {
        public:

            /// Map type
            enum Type {
                    /// Features Map + G2O
                    MAP_FEATURES,
            };

            /// overloaded constructor
            Map(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the map
            virtual const std::string& getName() const = 0;

            /// Virtual descrutor
            virtual ~Map() {}


            /// get all visible features
            virtual std::vector<MapFeature>& getVisibleFeatures(const Mat34& cameraPose) = 0;

            /// Add new features and camera pose (initial guess) to the map
            /// Position of features in relation to camera pose
            virtual void addFeatures(const std::vector<RGBDFeature>& features, const Mat34& cameraPose) = 0;

            /// Get all features
            virtual std::vector<MapFeature>& getAllFeatures(void) = 0;

            /// Get feature position
            virtual Vec3 getFeaturePosition(unsigned int id) = 0;

            /// get current pose of the sensor
            virtual Mat34 getCurrentPose(void) = 0;

            /// start optimization thread
            virtual void startOptimizationThread(unsigned int iterNo) = 0;

            /// Wait for optimization thread to finish
            virtual void finishOptimization() = 0;

        protected:
            /// Map type
            Type type;

            /// Map name
            const std::string name;
    };
};

#endif // _MAP_H_
