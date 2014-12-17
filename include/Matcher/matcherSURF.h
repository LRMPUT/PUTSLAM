/** @file matcherSURF.h
 *
 * implementation -
 *
 */

#ifndef MATCHERSURF_H_INCLUDED
#define MATCHERSURF_H_INCLUDED

#include "matcher.h"
#include <iostream>
#include <memory>

namespace putslam {
/// create a single matcher (SURF)
Matcher* createMatcherSURF(void);
}
;

using namespace putslam;

/// Tracker implementation
class MatcherSURF: public Matcher {
public:
	/// Pointer
	typedef std::unique_ptr<MatcherSURF> Ptr;

	/// Construction
	MatcherSURF(void);

	/// Construction
	MatcherSURF(const std::string _name) :
			Matcher(_name) {
	}

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

};

#endif // MATCHERSURF_H_INCLUDED
