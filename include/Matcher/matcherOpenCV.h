/** @file matcherSURF.h
 *
 * implementation -
 *
 */

#ifndef MATCHERSURF_H_INCLUDED
#define MATCHERSURF_H_INCLUDED

#include "matcher.h"
/** @file matcherOpenCV.h
 *
 * \brief The matching methods implemented for detectors and descriptors in OpenCV
 * \author Michal Nowicki
 *
 */
#include <iostream>
#include <memory>

namespace putslam {
/// create a single matcher OpenCV
Matcher* createMatcherOpenCV(void);
}
;

using namespace putslam;

/// Tracker implementation
class MatcherOpenCV: public Matcher {
public:
	enum type{SURF, SIFT, ORB, FAST, LDB};

	/// Pointer
	typedef std::unique_ptr<MatcherOpenCV> Ptr;

	/// Construction
	MatcherOpenCV(void);

	/// Construction
	MatcherOpenCV(type detector, type descriptor);

	/// Construction
	MatcherOpenCV(const std::string _name) : Matcher(_name) {};


	/// Name of the matcher
	virtual const std::string& getName() const;

	/// Detect features
	virtual std::vector<cv::KeyPoint> detectFeatures(cv::Mat rgbImage);

	/// Describe features
	virtual cv::Mat describeFeatures(cv::Mat rgbImage,
			std::vector<cv::KeyPoint> features);

	/// Perform matching
	virtual std::vector<cv::DMatch> performMatching(cv::Mat prevDescriptors,
			cv::Mat descriptors);

//
//        /// Returns current set of features
//        virtual const ImageFeature::Seq& getFeatures(void) const;
//
	/// Reset tracking and find new set of features
	virtual void reset();

private:
	/// Type of OpenCV matcher
	type detectorType;
	type descriptorType;


};

#endif // MATCHERSURF_H_INCLUDED
