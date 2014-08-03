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
    TransformEst* createKabschEstimator(void);
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
            const std::string& getName() const;

            /// compute transformation using two set of keypoints
            Mat34& computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB);

            /// Virtual descrutor
            virtual ~KabschEst() {}

        private:
            /// Tracker name
            const std::string name;
    };
};

#endif // _KABSCHEST_H_
