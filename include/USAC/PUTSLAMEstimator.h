#ifndef PUTSLAMESTIMATOR_H
#define PUTSLAMESTIMATOR_H

#include <iostream>
#include <fstream>
#include <string>
#include "ConfigParamsPUTSLAM.h"
#include "USAC.h"
#include "../include/TransformEst/g2oEst.h"
#include "../include/RGBD/RGBD.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

class PUTSLAMEstimator : public USAC<PUTSLAMEstimator>
{
public:
	struct parameters {
		int verbose;
		int errorVersion, errorVersionVO, errorVersionMap;
		double inlierThresholdEuclidean, inlierThresholdReprojection, inlierThresholdMahalanobis;
		double minimalInlierRatioThreshold;
		int usedPairs;
		int iterationCount;
	};
	enum ERROR_VERSION { EUCLIDEAN_ERROR, REPROJECTION_ERROR, EUCLIDEAN_AND_REPROJECTION_ERROR, MAHALANOBIS_ERROR, ADAPTIVE_ERROR };
	enum TransfEstimationType {
		UMEYAMA, G2O
	};

public:
	inline bool		 initProblem(const ConfigParamsPUTSLAM& cfg);

public:
	PUTSLAMEstimator(const parameters &_RANSACParameters, const cv::Mat &_cameraMatrix)
	{
		this->cameraMatrix = _cameraMatrix;
		this->RANSACParams = _RANSACParameters;
	};
	~PUTSLAMEstimator()
	{
	};

public:
	// ------------------------------------------------------------------------
	// problem specific functions
	void		 cleanupProblem();
	unsigned int generateMinimalSampleModels();
	bool		 generateRefinedModel(std::vector<unsigned int>& sample, const unsigned int numPoints,
		bool weighted = false, double* weights = NULL);
	bool		 validateSample();
	bool		 validateModel(unsigned int modelIndex);
	bool		 evaluateModel(unsigned int modelIndex, unsigned int* numInliers, unsigned int* numPointsTested);
	void		 testSolutionDegeneracy(bool* degenerateModel, bool* upgradeModel);
	unsigned int upgradeDegenerateModel();
	void		 findWeights(unsigned int modelIndex, const std::vector<unsigned int>& inliers,
		unsigned int numInliers, double* weights);
	void		 storeModel(unsigned int modelIndex, unsigned int numInliers);



// MF added:
public:
	
	void initPUTSLAMData(
		const std::vector<Eigen::Vector3f> &_prevFeatures,
		const std::vector<Eigen::Vector3f> &_features,
		const std::vector<cv::DMatch> &_matches,
		std::vector<cv::DMatch> &_bestInlierMatches,
		Eigen::Matrix4f &_bestTransformationModel,
		float &_bestInlierRatio
		)
	{
		this->prevFeatures = _prevFeatures;
		this->features = _features;
		this->matches = _matches;
		this->bestInlierMatches = _bestInlierMatches;
		this->bestTransformationModel = _bestTransformationModel;
		this->bestInlierRatio = _bestInlierRatio;
	}

	// generate model
	bool computeTransformationModel(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		Eigen::Matrix4f &transformationModel, TransfEstimationType usedType
	);

	// evaluate model
	float computeInlierRatioEuclidean(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> &modelConsistentMatches
	);

private:
	cv::Mat cameraMatrix;
	parameters RANSACParams;

	// generate sample
	std::vector<cv::DMatch> convertUSACSamplesToPUTSLAMSamples(std::vector<unsigned int> samplesFromUSAC, const std::vector<cv::DMatch> matches);
	std::vector<cv::DMatch> randomMatches;

	// generate model
	std::vector<Eigen::Vector3f> prevFeatures;
	std::vector<Eigen::Vector3f> features;
	std::vector<cv::DMatch> matches;
	Eigen::Matrix4f transformationModel;
	TransfEstimationType usedType;

	// validate model
	bool checkModelFeasibility(Eigen::Matrix4f transformationModel);

	// evaluate model
	std::vector<cv::DMatch> modelConsistentMatches;

	float computeInlierRatioMahalanobis(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> &modelConsistentMatches);

	float computeInlierRatioReprojection(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> &modelConsistentMatches);

	float computeInlierRatioEuclideanAndReprojection(
		const std::vector<Eigen::Vector3f> prevFeatures,
		const std::vector<Eigen::Vector3f> features,
		const std::vector<cv::DMatch> matches,
		const Eigen::Matrix4f transformationModel,
		std::vector<cv::DMatch> &modelConsistentMatches);

	unsigned int convertInlierRatioToNumberOfInliers(float inlierRatio);

	// store model
	Eigen::Matrix4f bestTransformationModel;
	float bestInlierRatio;
	std::vector<cv::DMatch> bestInlierMatches;

	// test model degeneracy

	// upgrade degenerated model

	// find weights

	// generate refined model
};


// ============================================================================================
// initProblem: initializes problem specific data and parameters
// this function is called once per run on new data
// ============================================================================================
bool PUTSLAMEstimator::initProblem(const ConfigParamsPUTSLAM& cfg)
{
	return true;
}


// ============================================================================================
// cleanupProblem: release any temporary problem specific data storage 
// this function is called at the end of each run on new data
// ============================================================================================
void PUTSLAMEstimator::cleanupProblem()
{
}


#endif