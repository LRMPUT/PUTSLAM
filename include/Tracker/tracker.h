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
            Tracker(const std::string _name) : name(_name){}

            /// Name of the tracker
            virtual const std::string& getName() const = 0;

            /// Returns the current set of features
            virtual const ImageFeature::Seq& getFeatures(void) const = 0;

            /// Reset tracking and find new set of features
            virtual void reset() = 0;

            /// Run tracking thread
            virtual void run() = 0;

            /// Virtual descrutor
            virtual ~Tracker() {}

        protected:
            /// A set of 2D features
            ImageFeature::Seq features;

            /// Tracker name
            const std::string name;
    };
};

#endif // _TRACKER_H_
