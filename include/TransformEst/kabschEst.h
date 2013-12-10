/** @file kabschEst.h
 *
 * Kabsch algorithm interface (transformation estimation)
 *
 */

#ifndef _KABSCHEST_H_
#define _KABSCHEST_H_

#include "transformEst.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single transform estimator (Kabsch)
    TransformEst* createKapschEstimator(void);
};

namespace putslam {
    /// Kabsch Algorithm interface
    class KabschEst : public TransformEst {
        public:
            /// Pointer
            typedef std::unique_ptr<KabschEst> Ptr;

            /// Construction
            KabschEst(void);

            /// Name of the Transformation estimator
            virtual const std::string& getName() const;

            /// Returns current transformation
            virtual const Mat34& getTransformation(void) const;

            /// compute transformation using two set of keypoints
            virtual void computeTransformation(ImageFeature::Seq& keypointA, ImageFeature::Seq& keypointB);

            /// Virtual descrutor
            virtual ~KabschEst() {}

        private:
            /// Estimated transformation
            Mat34 transformation;
            /// Tracker name
            const std::string name;
    };
};

#endif // _KABSCHEST_H_
