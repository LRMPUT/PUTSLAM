#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "../include/TransformEst/RANSAC.h"
#include "../include/USAC/USAC_wrapper.h"

/*
Cześć,

Przygotowałem dane do przetestowania uSACa.W załączniku są one spakowane w zipie.Znajdują się tam pliki o nazwach odpowiadających timestampą dla datasetu z freiburga(możesz sobie ręcznie zmienić) oraz różnych rozszerzeniach :
-.features to plik z cechami z danej chwili.Każda cecha to 1 linia.W jednej linii jest następujący format : id u v x y z
Jeśli x y z są ~0 to znaczy, że nie było poprawnego pomiaru głębi
- .matches zawierające dopasowania.Każda linia to 1 dopasowanie i składa się z id cechy z poprzedniej klatki od id cechy z aktualnej klatki.Plików.matches jest o 1 mniej niż plików.features
- .ransac zawiera estymowaną transformację z RANSACa.Jest to pozycja nowego układu w układzie starszym.

Jakbyś miał wątpliwości to daj znać : )
*/

void TEST_Ransac(
	std::vector<Eigen::Vector3f> previous_features,
	std::vector<Eigen::Vector3f> current_features,
	std::vector<cv::DMatch> matches
	)
{
	/*
	<!--RANSAC settings
	verbose : 0->no debug
	1->print final result
	2->print every iteration
	errorVersionVO : 0->Euclidean error
	1->Reprojection error
	2->Euclidean and Reprojection error
	errorVersionMap : 0->Euclidean error
	1->Reprojection error
	2->Euclidean and Reprojection error
	3->Mahalanobis distance
	4->Adaptive Euclidean error
	inlierThresholdEuclidean : maximal Euclidean error of inlier in RANSAC(in meters)
	inlierThresholdReprojection : maximal reprojection error of inlier in RANSAC(in pixels)
	minimalInlierRatioThreshold : minimal ratio of inliers to assume that estimated best model is correct(values : from 0 to 1)
	usedPairs : numbers of pairs used to create model in RANSAC iteration
	-->
	<RANSAC
		verbose = "0"
		errorVersionVO = "0"
		errorVersionMap = "0"
		inlierThresholdEuclidean = "0.02"
		inlierThresholdReprojection = "2.0"
		inlierThresholdMahalanobis = "0.0002"
		minimalInlierRatioThreshold = "0.1"
		usedPairs = "3"
	>
	< / RANSAC>
	*/

	//struct parameters {
	//	int verbose;
	//	int errorVersion, errorVersionVO, errorVersionMap;
	//	double inlierThresholdEuclidean, inlierThresholdReprojection, inlierThresholdMahalanobis;
	//	double minimalInlierRatioThreshold;
	//	int usedPairs;
	//	int iterationCount;
	//};

	RANSAC::parameters RANSACParams;

	RANSACParams.verbose = 1;
	RANSACParams.errorVersion = 0;	// ?
	RANSACParams.errorVersionVO = 0;
	RANSACParams.errorVersionMap = 0;
	RANSACParams.inlierThresholdEuclidean = 0.02;
	RANSACParams.inlierThresholdReprojection = 2.0;
	RANSACParams.inlierThresholdMahalanobis = 0.0002;
	RANSACParams.minimalInlierRatioThreshold = 0.1;	// ?
	RANSACParams.usedPairs = 3;
	RANSACParams.iterationCount = 0;	// ? nieważne

	cv::Mat cameraMatrixMat = cv::Mat::zeros(3, 3, CV_32FC1);

	RANSAC ransac(
		RANSACParams,
		cameraMatrixMat
	);

	std::vector<cv::DMatch> inlierMatches;

	Eigen::Matrix4f estimatedTransformation = ransac.estimateTransformation(
		previous_features,
		current_features,
		matches,
		inlierMatches
	);

	std::cout << "RANSAC:" << std::endl;
	std::cout << estimatedTransformation << std::endl;
}

class Features
{
	struct Feature
	{
		int id;
		float u;
		float v;
		float x;
		float y;
		float z;

		void Show(void)
		{
			std::cout << "Id: " << this->id << ", u: " << this->u << ", v: " << this->v << std::endl;
		}
	};

	private:
		std::vector<Feature> features;

	public:
	void LoadFromFile(std::string filename)
	{
		std::ifstream in_stream(filename);
		if (!in_stream)
		{
			std::cout << "Error opening the file" << std::endl;
		}

		while (!in_stream.eof())
		{
			Feature feature;
			in_stream >> feature.id >> feature.u >> feature.v >> feature.x >> feature.y >> feature.z;
			this->features.push_back(feature);
		}

		in_stream.close();
	}

	std::vector<Eigen::Vector3f> ConvertToEigen(void)
	{
		std::vector<Eigen::Vector3f> featuresEigen;

		for (auto& feature : features)
		{
			Eigen::Vector3f featureEigen;
			featureEigen[0] = feature.x;
			featureEigen[1] = feature.y;
			featureEigen[2] = feature.z;

			featuresEigen.push_back(featureEigen);
		}

		return featuresEigen;
	}

	void Show(void)
	{
		for (auto& feature : this->features)
		{
			feature.Show();
		}
	}
};

std::vector<cv::DMatch> loadMatchesFromFile(std::string filename)
{
	std::ifstream in_stream(filename);
	if (!in_stream)
	{
		std::cout << "Error opening the file" << std::endl;
	}

	std::vector<cv::DMatch> matches;

	while (!in_stream.eof())
	{
		cv::DMatch newMatch;
		in_stream >> newMatch.queryIdx >> newMatch.trainIdx;
		newMatch.imgIdx = 0;
		newMatch.distance = 0.0;
		matches.push_back(newMatch);
	}

	in_stream.close();

	return matches;
}

Eigen::Matrix4f loadTransformationFromFile(std::string filename)
{
	std::ifstream in_stream(filename);
	if (!in_stream)
	{
		std::cout << "Error opening the file" << std::endl;
	}

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			in_stream >> transformation(i, j);
		}
	}

	in_stream.close();

	return transformation;
}

void TEST_USAC(
		std::vector<Eigen::Vector3f> previous_features,
		std::vector<Eigen::Vector3f> current_features,
		std::vector<cv::DMatch> matches
		)
{
	PUTSLAMEstimator::parameters RANSACParams;

	RANSACParams.verbose = 1;
	RANSACParams.errorVersion = 0;	// ?
	RANSACParams.errorVersionVO = 0;
	RANSACParams.errorVersionMap = 0;
	RANSACParams.inlierThresholdEuclidean = 0.02;
	RANSACParams.inlierThresholdReprojection = 2.0;
	RANSACParams.inlierThresholdMahalanobis = 0.0002;
	RANSACParams.minimalInlierRatioThreshold = 0.1;	// ?
	RANSACParams.usedPairs = 3;
	RANSACParams.iterationCount = 0;	// ? nieważne

	cv::Mat cameraMatrixMat = cv::Mat::zeros(3, 3, CV_32FC1);

	RANSAC_USAC ransac(
			RANSACParams,
			cameraMatrixMat
	);

	std::vector<cv::DMatch> inlierMatches;

	Eigen::Matrix4f estimatedTransformation = ransac.estimateTransformation(
		previous_features,
		current_features,
		matches,
		inlierMatches
	);

	std::cout << "RANSAC_USAC:" << std::endl;
	std::cout << estimatedTransformation << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << "Hello world from USAC demo!" << std::endl;

	std::string directory  = "..//..//resources//USAC//";

	std::vector<cv::DMatch> matches = loadMatchesFromFile(directory + "1311868164.6703196.matches");
	std::cout << "Matches: " << std::endl;
	for (auto& match : matches)
	{
		std::cout << match.queryIdx << ", " << match.trainIdx << ", " << match.distance << std::endl;
	}

	Features features_previous;
	features_previous.LoadFromFile(directory + "1311868164.5034142.features");
	std::cout << "Features previous: " << std::endl;
	features_previous.Show();

	Features features_current;
	features_current.LoadFromFile(directory +  "1311868164.6703196.features");
	std::cout << "Features previous: " << std::endl;
	features_current.Show();

	std::vector<Eigen::Vector3f> features_previous_eigen = features_previous.ConvertToEigen();
	std::vector<Eigen::Vector3f> features_current_eigen = features_current.ConvertToEigen();
	std::cout << "Features previous Eigen: " << std::endl;
	for (auto& feature : features_previous_eigen)
	//for (std::vector<Eigen::Vector3f>::iterator itr = features_previous_eigen.begin(); itr != features_previous_eigen.end(); ++itr)
	{
		std::cout << feature[0] << ", " << feature[1] << ", " << feature[2];
		std::cout << std::endl;
		//std::cout << (*itr)[0] << ", " << (*itr)[1] << ", " << (*itr)[2] << std::endl;
	}

	Eigen::Matrix4f transformation = loadTransformationFromFile(directory + "1311868164.6703196.ransac");

	std::cout << "Transformation: " << std::endl << transformation << std::endl;

	TEST_Ransac(features_previous_eigen, features_current_eigen, matches);
	TEST_USAC(features_previous_eigen, features_current_eigen, matches);

	return 1;
}
