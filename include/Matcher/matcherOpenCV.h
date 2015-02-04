/** @file matcherOpenCV.h
 *
 * \brief The matching methods implemented for detectors and descriptors in OpenCV
 * \author Michal Nowicki
 *
 */

#ifndef MATCHERSURF_H_INCLUDED
#define MATCHERSURF_H_INCLUDED

#include "matcher.h"
#include <iostream>
#include <memory>

namespace putslam {
/// create a single matcher OpenCV
Matcher* createMatcherOpenCV(void);
Matcher* createMatcherOpenCV(const std::string _parametersFile);
};



using namespace putslam;

/// Tracker implementation
class MatcherOpenCV: public Matcher {
public:

	/// Pointer
	typedef std::unique_ptr<MatcherOpenCV> Ptr;

	/// Constructors
	MatcherOpenCV(void);
	MatcherOpenCV(const std::string _parametersFile);
	MatcherOpenCV(const std::string _name, const std::string _parametersFile);


	// Destructor
	~MatcherOpenCV(void);

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

private:
	/// Detector and descriptor
	std::unique_ptr<cv::FeatureDetector> featureDetector;
	std::unique_ptr<cv::DescriptorExtractor> descriptorExtractor;
	std::unique_ptr<cv::BFMatcher> matcher;

	/// Method to initialize variables in all constructors
	void initVariables();
};

#endif // MATCHERSURF_H_INCLUDED
