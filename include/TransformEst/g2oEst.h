/** @file g2oEst.h
 *
 * g2o 4 trans estimation interface (transformation estimation)
 *
 */

#ifndef _G2OEST_H_
#define _G2OEST_H_

#include "transformEst.h"
#include "unscented.h"
#include "../PoseGraph/graph_g2o.h"
#include <iostream>
#include <memory>

namespace putslam {
    /// create a single transform estimator (g2o)
    TransformEst* createG2OEstimator(void);
};

typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace putslam {
    /// G2O Algorithm interface
    class G2OEst : public TransformEst {
        public:
            /// Pointer
            typedef std::unique_ptr<G2OEst> Ptr;

            /// Construction
            G2OEst(void);

            /// Name of the Transformation estimator
            const std::string& getName() const;

            /// compute transformation using two set of keypoints
            Mat34& computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB);

            /// compute transformation using two set of keypoints
            Mat34& computeTransformation(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation);

            /// Compute uncertainty matrix [6x6] (fi,psi,theta,x,y,z)
            const Mat66& computeUncertainty(const Eigen::MatrixXd& setA, std::vector<Mat33>& setAUncertainty, const Eigen::MatrixXd& setB, std::vector<Mat33>& setBUncertainty, Mat34& transformation);

            ///computes information matrix from hessian using unscented transform
            static Mat66 computeInformationMatrix(const Mat66& Hessian, const Mat34& transformation);

            static Eigen::Isometry3f v2t(const Vector6f& t);

            static Vector6f t2v(const Eigen::Isometry3f& t);

            template <class T>
            static bool isNan(const T& m){
              for (int i=0; i< m.rows(); i++) {
                for (int j=0; j< m.cols(); j++) {
              float v = m(i,j);
              if ( isnan( v ) )
                return true;
                }
              }
              return false;
            }

            /// Virtual descrutor
            virtual ~G2OEst() {}

        private:
            /// Tracker name
            const std::string name;
            /// g2o graph
            PoseGraphG2O graph;
    };
};

#endif // _G2OEST_H_
