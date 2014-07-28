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

            /// Set Keypoints used for transformation estimation
            virtual void setInputKeypoints(ImageFeature::Seq& keypointA, ImageFeature::Seq& keypointB);

            /// compute transformation using two set of keypoints
            virtual const Mat34& computeTransformation(void);

            /// Virtual descrutor
            virtual ~KabschEst() {}

        private:
            /// Tracker name
            const std::string name;
    };
};

#endif // _KABSCHEST_H_
