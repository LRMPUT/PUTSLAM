/** @file USAC_wrapper.h
*
* \brief The robust estimation method to find transformation based on two sets of 3D features that uses USAC under the hood
* \author Michal Fularz
*
*/

#ifndef _USACWRAPPER_H_
#define _USACWRAPPER_H_

#include "PUTSLAMEstimator.h"

class RANSAC_USAC {
public:

	enum ERROR_VERSION { EUCLIDEAN_ERROR, REPROJECTION_ERROR, EUCLIDEAN_AND_REPROJECTION_ERROR, MAHALANOBIS_ERROR, ADAPTIVE_ERROR };

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

	cv::Mat cameraMatrix;
	PUTSLAMEstimator::parameters RANSACParams;

	//TODO move it up (it shouldn't be here. The object is created and destroyed at each iteration of the matching procedure)
	//DepthSensorModel sensorModel;

	enum TransfEstimationType {
		UMEYAMA, G2O
	};

	ConfigParamsPUTSLAM init_usac_configuration(void);

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