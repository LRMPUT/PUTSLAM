/** @file ICPEst.h
 *
 * Genaralized ICP algorithm interface (transformation estimation)
 *
 */

#ifndef _ICPEST_H_
#define _ICPEST_H_

#include "transformEst.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single transform estimator (ICP)
    TransformEst* createICPEstimator(void);
};

namespace putslam {
    /// ICP Algorithm interface
    class ICPEst : public TransformEst {
        public:
            /// Pointer
            typedef std::unique_ptr<ICPEst> Ptr;

            /// Construction
            ICPEst(void);

            /// Name of the Transformation estimator
            virtual const std::string& getName() const;

            /// Set point clouds used for transformation estimation
            virtual void setInputClouds(PointCloud cloudA, PointCloud cloudB);

            /// compute transformation using two set of keypoints
            virtual const Mat34& computeTransformation(void);

            /// Virtual descrutor
            virtual ~ICPEst() {}

        private:
            /// Estimated transformation
            Mat34 transformation;
            /// Tracker name
            const std::string name;
    };
};

#endif // _ICPEST_H_
