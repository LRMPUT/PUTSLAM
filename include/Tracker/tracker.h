/** @file tracker.h
 *
 * the interface of 2D image feature tracker
 *
 */

#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Grabber interface
    class Tracker {
        public:

            /// Overloaded constructor
            Tracker(const std::string _name) : name(_name), frame_id(0){}

            /// Name of the tracker
            virtual const std::string& getName() const = 0;

            /// Returns the current set of features
            virtual const ImageFeature::Seq& getFeatures(void) const = 0;

            /// Reset tracking and find new set of features
            virtual void reset() = 0;

            /// Run single tracking iteration
            virtual bool track(const SensorFrame& next_frame) = 0;

            /// Compute homogenous transformation
            virtual const Mat34& computeTransform(void) = 0;

            /// get Vertex: set of Keypoints/ point Cloud and sensor/robot pose
            virtual const VertexSE3& getVertex(void) = 0;

            /// Virtual descrutor
            virtual ~Tracker() {}

        protected:
            /// A set of 2D features
            ImageFeature::Seq features;

            /// Tracker name
            const std::string name;

            /// Frame id
            uint_fast32_t frame_id;

            /// Computed homogenous transformation
            Mat34 transformation;

            /// keypoint: robot/sensor pose + point cloud + features
            VertexSE3 keypoint;
    };
};

#endif // _TRACKER_H_
