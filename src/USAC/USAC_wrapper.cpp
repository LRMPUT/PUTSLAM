#include "../include/USAC/USAC_wrapper.h"

//#include "../include/TransformEst/g2oEst.h"
//#include "../include/RGBD/RGBD.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "../include/USAC/PUTSLAMEstimator.h"

#include <time.h>		// for srand

RANSAC_USAC::RANSAC_USAC(PUTSLAMEstimator::parameters _RANSACParameters, cv::Mat _cameraMatrix)
{
	//RANSAC::RANSAC(RANSAC::parameters _RANSACParameters, cv::Mat _cameraMatrix) : sensorModel("fileModel.xml") {
	srand(time(0));

	// MF:
	// why this does not compile (had to add last two parameters)
	_RANSACParameters.iterationCount = computeRANSACIteration(0.20, 0.99, _RANSACParameters.usedPairs);

	this->putslamest = new PUTSLAMEstimator(_RANSACParameters, _cameraMatrix);
}

RANSAC_USAC::~RANSAC_USAC()
{
	this->putslamest->cleanupProblem();
	delete this->putslamest;
}

inline int RANSAC_USAC::computeRANSACIteration(
	double inlierRatio,
	double successProbability,
	int numberOfPairs
	)
{
	return (log(1 - successProbability)
		/ log(1 - pow(inlierRatio, numberOfPairs)));
}

void RANSAC_USAC::remove_matches_with_invalid_depth(
	std::vector<cv::DMatch> &matches,
	const std::vector<Eigen::Vector3f> &prevFeatures,
	const std::vector<Eigen::Vector3f> &features
	)
{
	// Remove matches with features containing invalid depth
	// TODO: It is slow due to the vector rebuilds
	for (std::vector<cv::DMatch>::iterator it = matches.begin();
		it != matches.end();) {
		int prevId = it->queryIdx, id = it->trainIdx;

		if (prevFeatures[prevId].hasNaN() || features[id].hasNaN()
			|| prevFeatures[prevId][2] < 0.1 || prevFeatures[prevId][2] > 6
			|| features[id][2] < 0.1 || features[id][2] > 6)
			it = matches.erase(it);
		else
			++it;
	}
}

ConfigParamsPUTSLAM RANSAC_USAC::init_usac_configuration(void)
{
	ConfigParamsPUTSLAM cfg;

	cfg.common.confThreshold = 0.99;
	cfg.common.minSampleSize = 3;
	// IMPORTANT:
	cfg.common.inlierThreshold = 0.02;
	cfg.common.maxHypotheses = 850000;
	cfg.common.maxSolutionsPerSample = 1;
	cfg.common.prevalidateSample = false;
	cfg.common.prevalidateModel = false;
	// MF
	// different for each sample
	cfg.common.numDataPoints = 133;
	cfg.common.testDegeneracy = false;
	cfg.common.randomSamplingMethod = USACConfig::SAMP_UNIFORM;
	cfg.common.verifMethod = USACConfig::VERIF_STANDARD;
	cfg.common.localOptMethod = USACConfig::LO_NONE;

	cfg.prosac.maxSamples = 100000;
	cfg.prosac.beta = 0.99;
	cfg.prosac.minStopLen = 20;
	cfg.prosac.nonRandConf = 0.99;
	// TODO: correct
	cfg.prosac.sortedPointsFile = "F:\\Amin\\Desktop\\USAC\\data\\fundmatrix\\test1\\sorting.txt";

	cfg.sprt.tM = 200.0;
	cfg.sprt.mS = 2.38;
	cfg.sprt.delta = 0.05;
	cfg.sprt.epsilon = 0.15;

	cfg.losac.innerSampleSize = 15;
	cfg.losac.innerRansacRepetitions = 5;
	cfg.losac.thresholdMultiplier = 2.0;
	cfg.losac.numStepsIterative = 4;

	return cfg;
}

// TODO: MISSING:
// - test of minimal inlierRatio of bestModel
Eigen::Matrix4f RANSAC_USAC::estimateTransformation(
	std::vector<Eigen::Vector3f> prevFeatures,
	std::vector<Eigen::Vector3f> features, 
	std::vector<cv::DMatch> matches,
	std::vector<cv::DMatch> & bestInlierMatches) 
{
	Eigen::Matrix4f bestTransformationModel = Eigen::Matrix4f::Identity();
	float bestInlierRatio = 0.0;

	// MF addition (just packed some code into a function):
	remove_matches_with_invalid_depth(matches, prevFeatures, features);

	if (this->putslamest->RANSACParams.verbose > 0)
		std::cout << "RANSAC: matches.size() = " << matches.size() << std::endl;

	// TODO: DO IT NICER!
	if (matches.size() < 8) {
		return Eigen::Matrix4f::Identity();
	}

	// MF:
	// instead of own RANSAC iteration we are using USAC
	this->USAC_run(
		prevFeatures,
		features,
		matches,
		bestInlierMatches,
		bestTransformationModel,
		bestInlierRatio
	);

	// MF:
	// Reestimate from inliers - done in USAC_run (at the end)

	// Test the number of inliers
	if (bestInlierRatio < this->putslamest->RANSACParams.minimalInlierRatioThreshold) {
		bestTransformationModel = Eigen::Matrix4f::Identity();
	}

	// Test for minimal inlierRatio of bestModel
	if (this->putslamest->RANSACParams.verbose > 0) {
		std::cout << "RANSAC best model : inlierRatio = "
			<< bestInlierRatio * 100.0 << "%" << std::endl;
		std::cout << "RANSAC best model : " << std::endl
			<< bestTransformationModel << std::endl;
	}
	return bestTransformationModel;
}

int RANSAC_USAC::USAC_run(
	const std::vector<Eigen::Vector3f> &prevFeatures,
	const std::vector<Eigen::Vector3f> &features,
	const std::vector<cv::DMatch> &matches,
	std::vector<cv::DMatch> &bestInlierMatches,
	Eigen::Matrix4f &bestTransformationModel,
	float &bestInlierRatio
)
{
	// MF:
	// already done in RANSAC constructor
	// seed random number generator
	//srand((unsigned int)time(NULL));
	
	// MF TODO:
	// pass all the provided data to appropriate USAC structure
	// initialize the PUTSLAM problem
	ConfigParamsPUTSLAM cfg = this->init_usac_configuration();
	// fill the cfg structure
	
	putslamest->initParamsUSAC(cfg);

	// read in prosac data if required
	putslamest->initDataUSAC(cfg);
	putslamest->initProblem(cfg);

	// put all the data into PUTSLAMEstimator class
	putslamest->initPUTSLAMData(
		prevFeatures, 
		features, 
		matches
	);

	if (!putslamest->solve())
	{
		std::cout << "USAC_Wrapper: " << "putslam->solve()" << " failed." << std::endl;
		return (EXIT_FAILURE);
	}

	std::cout << "USAC_Wrapper: " << "putslam->solve()" << " succeed." << std::endl;

	// get all the data from PUTSLAMEstimator class
	putslamest->deInitPUTSLAMData(
		bestInlierMatches,
		bestTransformationModel,
		bestInlierRatio
	);

	// additional logic from PUTSLAM (reestimation of the model)
	// Reestimate from inliers
	// MF TODO:
	// is this correct? Why using euclidean inlier ratio?
	/*
	putslamest->computeTransformationModel(
		prevFeatures, 
		features, 
		bestInlierMatches,
		bestTransformationModel, 
		putslamest->UMEYAMA
	);
	std::vector<cv::DMatch> newBestInlierMatches;
	putslamest->computeInlierRatioEuclidean(
		prevFeatures, 
		features, 
		bestInlierMatches,
		bestTransformationModel, 
		newBestInlierMatches
	);
	newBestInlierMatches.swap(bestInlierMatches);
	*/

	return (EXIT_SUCCESS);
}
