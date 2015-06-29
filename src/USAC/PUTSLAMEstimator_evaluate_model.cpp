#include "../include/USAC/PUTSLAMEstimator.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "../include/RGBD/RGBD.h"

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

//// COPIED FROM PUTSLAM:
/**
* Method used to compute the inlierRatio based on 3D Euclidean error:
*
* prevFeatures				-- 	first set of 3D features
* features					--	second set of 3D features
* matches					-- 	vector of matches to be determined as inliers or outliers
* transformationModel		--	transformation used in evaluation
* modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
*/
float PUTSLAMEstimator::computeInlierRatioEuclidean(
	const std::vector<Eigen::Vector3f> prevFeatures,
	const std::vector<Eigen::Vector3f> features,
	const std::vector<cv::DMatch> matches,
	const Eigen::Matrix4f transformationModel,
	std::vector<cv::DMatch> &modelConsistentMatches) {
	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f R = transformationModel.block<3, 3>(0, 0);
	Eigen::Vector3f t = transformationModel.block<3, 1>(0, 3);

	int inlierCount = 0;
	// For all matches
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
		it != matches.end(); ++it) {
		// Estimate location of feature from position one after transformation
		Eigen::Vector3f estimatedOldPosition = R * features[it->trainIdx] + t;

		// Compute residual error and compare it to inlier threshold
		float_type threshold = RANSACParams.inlierThresholdEuclidean;
		if (RANSACParams.errorVersion == ADAPTIVE_ERROR)
			threshold *= prevFeatures[it->queryIdx].z();
		if ((estimatedOldPosition - prevFeatures[it->queryIdx]).norm()
			< threshold) {
			inlierCount++;
			modelConsistentMatches.push_back(*it);
		}
	}

	std::cout << "Inlier count: " << inlierCount << std::endl;

	// Percent of correct matches
	return float(inlierCount) / matches.size();
}

/**
* Method used to compute the inlierRatio based on 3D Mahalanobis error:
*
* prevFeatures				-- 	first set of 3D features
* features					--	second set of 3D features
* matches					-- 	vector of matches to be determined as inliers or outliers
* transformationModel		--	transformation used in evaluation
* modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
*/
float PUTSLAMEstimator::computeInlierRatioMahalanobis(
	const std::vector<Eigen::Vector3f> prevFeatures,
	const std::vector<Eigen::Vector3f> features,
	const std::vector<cv::DMatch> matches,
	const Eigen::Matrix4f transformationModel,
	std::vector<cv::DMatch> &modelConsistentMatches) {
	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f R = transformationModel.block<3, 3>(0, 0);
	Eigen::Vector3f t = transformationModel.block<3, 1>(0, 3);

	int inlierCount = 0;
	// For all matches
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
		it != matches.end(); ++it) {
		// Estimate location of feature from position one after transformation
		Eigen::Vector3f estimatedOldPosition = R * features[it->trainIdx] + t;

		// Compute residual error and compare it to inlier threshold
		Mat33 cov;
		//sensorModel.computeCov(prevFeatures[it->queryIdx],cov);
		if (cov.determinant() != 0){
			double distMah = (estimatedOldPosition - prevFeatures[it->queryIdx]).transpose()*cov.cast<float>()*(estimatedOldPosition - prevFeatures[it->queryIdx]);
			/*std::cout << "vec: " << estimatedOldPosition.x() << " -> " << prevFeatures[it->queryIdx].x() <<"\n";
			std::cout << "vec: " << estimatedOldPosition.y() << " -> " << prevFeatures[it->queryIdx].y() <<"\n";
			std::cout << "vec: " << estimatedOldPosition.z() << " -> " << prevFeatures[it->queryIdx].z() <<"\n";
			std::cout << "distMah: " << distMah << " distEucl: " << (estimatedOldPosition - prevFeatures[it->queryIdx]).norm() << "\n";
			std::cout << "distMahThr: " << RANSACParams.inlierThresholdMahalanobis << "\n";
			getchar();*/
			if (distMah < RANSACParams.inlierThresholdMahalanobis) {
				inlierCount++;
				modelConsistentMatches.push_back(*it);
			}
		}
	}

	// Percent of correct matches
	return float(inlierCount) / matches.size();
}

/**
* Method used to compute the inlierRatio based on reprojection error:
*
* prevFeatures				-- 	first set of 3D features
* features					--	second set of 3D features
* matches					-- 	vector of matches to be determined as inliers or outliers
* transformationModel		--	transformation used in evaluation
* modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
*/
float PUTSLAMEstimator::computeInlierRatioReprojection(
	const std::vector<Eigen::Vector3f> prevFeatures,
	const std::vector<Eigen::Vector3f> features,
	const std::vector<cv::DMatch> matches,
	const Eigen::Matrix4f transformationModel,
	std::vector<cv::DMatch> &modelConsistentMatches) {

	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f R = transformationModel.block<3, 3>(0, 0);
	Eigen::Vector3f t = transformationModel.block<3, 1>(0, 3);

	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f Rinv = transformationModel.inverse().block<3, 3>(0, 0);
	Eigen::Vector3f tinv = transformationModel.inverse().block<3, 1>(0, 3);

	int inlierCount = 0;
	// For all matches
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
		it != matches.end(); ++it) {
		// Estimate location of feature from position one after transformation
		Eigen::Vector3f estimatedOldPosition = R * features[it->trainIdx] + t;
		Eigen::Vector3f estimatedNewPosition = Rinv * prevFeatures[it->queryIdx]
			+ tinv;

		// Now project both features
		cv::Point2f predictedNew = RGBD::point3Dto2D(estimatedNewPosition,
			cameraMatrix);
		cv::Point2f realNew = RGBD::point3Dto2D(features[it->trainIdx],
			cameraMatrix);

		cv::Point2f predictedOld = RGBD::point3Dto2D(estimatedOldPosition,
			cameraMatrix);
		cv::Point2f realOld = RGBD::point3Dto2D(prevFeatures[it->queryIdx],
			cameraMatrix);

		// Compute residual error and compare it to inlier threshold
		double error2D[2] { cv::norm(predictedNew - realNew), cv::norm(
			predictedOld - realOld) };

		// Compute residual error and compare it to inlier threshold
		if (error2D[0] < RANSACParams.inlierThresholdReprojection
			&& error2D[1] < RANSACParams.inlierThresholdReprojection) {

			inlierCount++;
			modelConsistentMatches.push_back(*it);
		}
	}

	// Percent of correct matches
	return float(inlierCount) / matches.size();
}

/**
* Method used to compute the inlierRatio based on eulidean and reprojection error (simultaneously):
*
* prevFeatures				-- 	first set of 3D features
* features					--	second set of 3D features
* matches					-- 	vector of matches to be determined as inliers or outliers
* transformationModel		--	transformation used in evaluation
* modelConsistentMatches	--  returns the matches that are considered inliers using currently evaluated model
*/
float PUTSLAMEstimator::computeInlierRatioEuclideanAndReprojection(
	const std::vector<Eigen::Vector3f> prevFeatures,
	const std::vector<Eigen::Vector3f> features,
	const std::vector<cv::DMatch> matches,
	const Eigen::Matrix4f transformationModel,
	std::vector<cv::DMatch> &modelConsistentMatches) {
	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f R = transformationModel.block<3, 3>(0, 0);
	Eigen::Vector3f t = transformationModel.block<3, 1>(0, 3);

	// Break into rotation (R) and translation (t)
	Eigen::Matrix3f Rinv = transformationModel.inverse().block<3, 3>(0, 0);
	Eigen::Vector3f tinv = transformationModel.inverse().block<3, 1>(0, 3);

	int inlierCount = 0;
	// For all matches
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
		it != matches.end(); ++it) {
		// Estimate location of feature from position one after transformation
		Eigen::Vector3f estimatedOldPosition = R * features[it->trainIdx] + t;
		Eigen::Vector3f estimatedNewPosition = Rinv * prevFeatures[it->queryIdx]
			+ tinv;

		// Compute error3D
		double error3D =
			(estimatedOldPosition - prevFeatures[it->queryIdx]).norm();

		// Now project both features
		cv::Point2f predictedNew = RGBD::point3Dto2D(estimatedNewPosition,
			cameraMatrix);
		cv::Point2f realNew = RGBD::point3Dto2D(features[it->trainIdx],
			cameraMatrix);

		cv::Point2f predictedOld = RGBD::point3Dto2D(estimatedOldPosition,
			cameraMatrix);
		cv::Point2f realOld = RGBD::point3Dto2D(prevFeatures[it->queryIdx],
			cameraMatrix);

		// Compute residual error and compare it to inlier threshold
		double error2D[2] { cv::norm(predictedNew - realNew), cv::norm(
			predictedOld - realOld) };

		//std::cout<<"ERROR : " << error3D << " " << error2D[0] << " " << error2D[1] << std::endl;

		// Compute residual error and compare it to inlier threshold
		if (error3D < RANSACParams.inlierThresholdEuclidean
			&& error2D[0] < RANSACParams.inlierThresholdReprojection
			&& error2D[1] < RANSACParams.inlierThresholdReprojection) {
			//			std::cout<<"3D position: " << estimatedOldPosition.transpose() << "\t" << prevFeatures[it->queryIdx].transpose() << std::endl;
			//			std::cout<<"3D position: " << estimatedNewPosition.transpose() << "\t" << features[it->trainIdx].transpose() << std::endl;
			//			std::cout<<"ERROR : " << error3D << " " << (estimatedNewPosition - features[it->trainIdx]).norm() <<" "<< error2D[0] << " " << error2D[1] << std::endl;

			inlierCount++;
			modelConsistentMatches.push_back(*it);
		}
	}

	// Percent of correct matches
	return float(inlierCount) / matches.size();
}

unsigned int PUTSLAMEstimator::convertInlierRatioToNumberOfInliers(float inlierRatio)
{
	unsigned int numberOfInliers = 0;

	numberOfInliers = inlierRatio * this->matches.size();

	return numberOfInliers;
}

// ============================================================================================
// evaluateModel: test model against all/subset of the data points
// ============================================================================================
bool PUTSLAMEstimator::evaluateModel(unsigned int modelIndex, unsigned int* numInliers, unsigned int* numPointsTested)
{
	// MF:
	// TODO - add model evaluation
	// simplest solution - no model evaluation
	// the HomogEstimator is usign something more complicated
	// the FundmatrixEstimator is usign something more complicated

	//// COPIED FROM PUTSLAM:
	// Evaluate the model
	if (RANSACParams.verbose > 1)
		std::cout << "RANSAC: evaluating the model" << std::endl;

	// Choose proper error computation version based on provided parameters
	float inlierRatio = 0.0;
	if ((RANSACParams.errorVersion == EUCLIDEAN_ERROR) ||
		(RANSACParams.errorVersion == ADAPTIVE_ERROR)){
		if (RANSACParams.verbose > 1) {
			std::cout << "Evaluation using Euclidean" << std::endl;
		}

		inlierRatio = computeInlierRatioEuclidean(
			this->prevFeatures,
			this->features,
			this->matches,
			this->transformationModel,
			this->modelConsistentMatches
		);
	}
	else if (RANSACParams.errorVersion == REPROJECTION_ERROR) {
		if (RANSACParams.verbose > 1) {
			std::cout << "Evaluation using Reprojection" << std::endl;
		}

		inlierRatio = computeInlierRatioReprojection(
			this->prevFeatures,
			this->features,
			this->matches,
			this->transformationModel,
			this->modelConsistentMatches
		);
	}
	else if (RANSACParams.errorVersion == EUCLIDEAN_AND_REPROJECTION_ERROR) {
		if (RANSACParams.verbose > 1) {
			std::cout << "Evaluation using EuclideanAndReprojection" << std::endl;
		}

		inlierRatio = computeInlierRatioEuclideanAndReprojection(
			this->prevFeatures, 
			this->features, 
			this->matches,
			this->transformationModel,
			this->modelConsistentMatches
		);
	}
	else if (RANSACParams.errorVersion == MAHALANOBIS_ERROR) {
		if (RANSACParams.verbose > 1) {
			std::cout << "Evaluation using Mahalanobis" << std::endl;
		}

		inlierRatio = computeInlierRatioMahalanobis(
			this->prevFeatures, 
			this->features, 
			this->matches,
			this->transformationModel,
			this->modelConsistentMatches
		);
	}
	else {
		std::cout << "RANSAC: incorrect error version" << std::endl;
	}

	this->inlierRatio = inlierRatio;
	std::cout << "Inlier ratio: " << this->inlierRatio << std::endl;

	// MF:
	// TODO:
	// both HomogEstimator and FundmatrixEstimator are using decision_threshold_sprt_
	// to finish earlier the evaluation and sets good_flag to false in such case
	*numPointsTested = this->matches.size();
	*numInliers = this->convertInlierRatioToNumberOfInliers(inlierRatio);

	bool good_flag = true;

	return good_flag;
}
