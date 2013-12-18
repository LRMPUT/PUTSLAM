/** @file transformEst.h
 *
 * Feature-based transformation estimation interface
 *
 */

#ifndef _TRANSFORMEST_H_
#define _TRANSFORMEST_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Transformation Estimator interface
    class TransformEst {
        public:

            /// Name of the estimator
            virtual const std::string& getName() const = 0;

            /// Grab image and/or point cloud
            virtual const Mat34& computeTransformation(void) = 0;

            /// Virtual descrutor
            virtual ~TransformEst() {}
    };
};

#endif // _TRANSFORMEST_H_

