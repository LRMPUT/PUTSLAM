/** @file trackerKLT.h
 *
 * implementation - Kanade-Lucas-Tomasi feature tracker
 *
 */

#ifndef TRACKERKLT_H_INCLUDED
#define TRACKERKLT_H_INCLUDED

#include "tracker.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single tracker (Kanade-Lucas-Tomasi)
    Tracker* createTrackerKLT(void);
};

using namespace putslam;

/// Tracker implementation
class TrackerKLT : public Tracker {
    public:
        /// Pointer
        typedef std::unique_ptr<TrackerKLT> Ptr;

        /// Construction
        TrackerKLT(void);

        /// Construction
        TrackerKLT(const std::string _name) : Tracker(_name){}

        /// Name of the tracker
        virtual const std::string& getName() const;

        /// Returns current set of features
        virtual const ImageFeature::Seq& getFeatures(void) const;

        /// Reset tracking and find new set of features
        virtual void reset();

        /// Run single tracking iteration
        virtual bool track(const SensorFrame& next_frame);

        /// Compute homogenous transformation
        virtual const RobotPose& computeTransform(void);

        /// get Vertex: set of Keypoints/ point Cloud and sensor/robot pose
        virtual const Vertex7D& getVertex(void);

    private:

};

#endif // TRACKERKLT_H_INCLUDED
