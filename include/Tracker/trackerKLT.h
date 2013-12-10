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

        /// Name of the tracker
        virtual const std::string& getName() const;

        /// Returns current set of features
        virtual const ImageFeature::Seq& getFeatures(void) const;

        /// Reset tracking and find new set of features
        virtual void reset();

        /// Run tracking thread
        virtual void run();

    private:
        /// A set of 2D features
        ImageFeature::Seq features;

        /// Tracker name
        const std::string name;
};

#endif // TRACKERKLT_H_INCLUDED
