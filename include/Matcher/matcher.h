/** @file tracker.h
 *
 * the interface of 2D image feature tracker
 *
 */

#ifndef _MATCHER_H_
#define _MATCHER_H_

#include "../Defs/putslam_defs.h"
#include <string>
#include <vector>
#include "opencv/cv.h"

namespace putslam {
/// Grabber interface
class Matcher {
public:

	/// Overloaded constructor
	Matcher(const std::string _name) :
			name(_name), frame_id(0) {
	}

	/// Name of the Matcher
	virtual const std::string& getName() const = 0;

	/// Detect features
	virtual std::vector<cv::KeyPoint> detectFeatures(cv::Mat rgbImage) = 0;

	/// Describe features
	virtual cv::Mat describeFeatures(cv::Mat rgbImage,
			std::vector<cv::KeyPoint> features) = 0;

	/// Perform matching
	virtual std::vector<cv::DMatch> performMatching(cv::Mat prevDescriptors,
			cv::Mat descriptors) = 0;

//
//            /// Returns the current set of features
//            virtual const ImageFeature::Seq& getFeatures(void) const = 0;
//
//            /// Reset tracking and find new set of features
//            virtual void reset() = 0;
//
	/// Run single match
	bool match(const SensorFrame& next_frame);

//
//            /// Compute homogenous transformation
//            virtual const RobotPose& computeTransform(void) = 0;
//
//            /// get Vertex: set of Keypoints/ point Cloud and sensor/robot pose
//            virtual const VertexSE3& getVertex(void) = 0;
//
//            /// Virtual descrutor
//            virtual ~Tracker() {}

protected:
//            /// A set of 2D features
//            ImageFeature::Seq features;
//
	/// Matcher name
	const std::string name;

	/// Frame id
	uint_fast32_t frame_id;
//
//            /// Computed homogenous transformation
//            RobotPose transformation;
//
//            /// keypoint: robot/sensor pose + point cloud + features
//            VertexSE3 keypoint;
};
}
;

#endif // _MATCHER_H_
