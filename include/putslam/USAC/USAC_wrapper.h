/** @file USAC_wrapper.h
*
* \brief The robust estimation method to find transformation based on two sets of 3D features that uses USAC under the hood
* \author Michal Fularz
*
*/

#ifndef _USACWRAPPER_H_
#define _USACWRAPPER_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "../include/USAC/PUTSLAMEstimator.h"

class RANSAC_USAC {

public:
	RANSAC_USAC(PUTSLAMEstimator::parameters _RANSACParameters, cv::Mat _cameraMatrix = cv::Mat());
	~RANSAC_USAC();

	/**
	* Method used to robustly estimate transformation from given two sets of
	* 3D features and potentially correct matches between those sets
	*
	* prevFeatures 	-- 	3D locations of features from the first set
	* features 		--	3D locations of features currently observed (2nd set)
	* matches			--	vector of indices of corresponding features
	* inlierMatches	--  vector of matches considered as inliers
	*/
	Eigen::Matrix4f estimateTransformation(
		std::vector<Eigen::Vector3f> prevFeatures,
		std::vector<Eigen::Vector3f> features,
		std::vector<cv::DMatch> matches,
		std::vector<cv::DMatch> & bestInlierMatches);

private:
	PUTSLAMEstimator* putslamest;

	ConfigParamsPUTSLAM init_usac_configuration(
			void
	);

	int USAC_run(
		const std::vector<Eigen::Vector3f> &prevFeatures,
		const std::vector<Eigen::Vector3f> &features,
		const std::vector<cv::DMatch> &matches,
		std::vector<cv::DMatch> &bestInlierMatches,
		Eigen::Matrix4f &bestTransformationModel,
		float &bestInlierRatio
	);

	inline int computeRANSACIteration(
		double inlierRatio,
		double successProbability,
		int numberOfPairs
	);

	void remove_matches_with_invalid_depth(
		std::vector<cv::DMatch> &matches,
		const std::vector<Eigen::Vector3f> &prevFeatures,
		const std::vector<Eigen::Vector3f> &features
	);

};

#endif // _USACWRAPPER_H_
