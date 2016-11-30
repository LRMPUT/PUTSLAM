#include "Matcher/MatchingOnPatches.h"
#include <stdio.h>
#include <opencv2/highgui.hpp>

MatchingOnPatches::MatchingOnPatches(int _patchSize, int _maxIter,
		double _minSqrtIncrement, int _verbose) {
	init(_patchSize, _maxIter, _minSqrtIncrement, _verbose);
}

MatchingOnPatches::MatchingOnPatches(parameters _parameters) {
	init(_parameters.patchSize, _parameters.maxIter, _parameters.minSqrtIncrement, _parameters.verbose);
}

std::vector<double> MatchingOnPatches::computePatch(cv::Mat img,
        double x, double y) {
	std::vector<double> patch;

	// Assert grayscale
	assertGrayscale(img);

	// subpix precision
	const double xLeft = int(x), yLeft = int(y);
	const double xSub = x - xLeft, ySub = y - yLeft;

	// From wiki: http://upload.wikimedia.org/math/9/b/4/9b4e1064436ecccd069ea238b656c063.png
	const double topLeft = (1.0 - xSub) * (1.0 - ySub);
	const double topRight = xSub * (1.0 - ySub);
	const double bottomLeft = (1.0 - xSub) * ySub;
	const double bottomRight = xSub * ySub;

	// compute patch
	unsigned char *input = (unsigned char*) (img.data);

	// Speed-up considerations
	const int yEnd = (int)y + params.halfPatchSize;
	const int xEnd = (int)x + params.halfPatchSize;

	for (int i = (int)y - params.halfPatchSize; i <= yEnd; i++) {
		for (int j = (int)x - params.halfPatchSize; j <= xEnd; j++) {

			double value = topLeft * input[img.step * i + j]
					+ topRight * input[img.step * (i) + j + 1]
					+ bottomLeft * input[img.step * (i + 1) + j]
					+ bottomRight * input[img.step * (i + 1) + j + 1];

			patch.push_back(value);
		}
	}

	return patch;
}

void MatchingOnPatches::computeGradient(cv::Mat img, double x,
        double y, Eigen::Matrix3f &InvHessian,
		std::vector<float> &gradientX, std::vector<float> &gradientY) {

	// Assert grayscale
	assertGrayscale(img);

	// compute patch
	unsigned char *input = (unsigned char*) (img.data);

	// Speed-up considerations
	const int yEnd = (int)y + params.halfPatchSize;
	const int xEnd = (int)x + params.halfPatchSize;


	Eigen::Matrix3f Hessian = Eigen::Matrix3f::Zero();

	for (int i = (int)y - params.halfPatchSize; i <= yEnd; i++) {
		for (int j = (int)x - params.halfPatchSize; j <= xEnd; j++) {

			Eigen::Vector3f J;
			J[0] = 0.5f
					* (float) (input[img.step * i + j + 1]
							- input[img.step * i + j - 1]);
			J[1] = 0.5f
					* (float)(input[img.step * (i + 1) + j]
							- input[img.step * (i - 1) + j]);
			J[2] = 1.0f;
			Hessian += J * J.transpose();

			gradientX.push_back(J[0]);
			gradientY.push_back(J[1]);

			if (params.verbose > 1) {
				std::cout<<"Values X: " << (int)input[img.step * i + j + 1] << " " << (int)input[img.step * i + j - 1] << std::endl;
				std::cout<<"Values Y: " << (int)input[img.step * (i + 1) + j] << " " << (int)input[img.step * (i - 1) + j] << std::endl;
				std::cout<<"Jacobian: " << J[0] << " " << J[1] << " " << J[2] << std::endl;
			}
		}
	}

	InvHessian = Hessian.inverse();
	if (params.verbose > 1) {
		std::cout<<"Hessian: " << std::endl << Hessian << std::endl;
		std::cout<<"InvHessian: " << std::endl << InvHessian << std::endl;
	}
}

bool MatchingOnPatches::optimizeLocation(cv::Mat oldImg,
		std::vector<double> oldPatch, cv::Mat newImg,
        double &newX, double &newY,
		std::vector<float> gradientX, std::vector<float> gradientY,
		Eigen::Matrix3f &InvHessian) {

	// TODO: See if it is only possible when minimum is reached
	if (std::isnan(InvHessian(0,0))) {
		return false;
	}

	//Cloning for drawing purposes
	cv::Mat drawImg = newImg.clone();

	// Convert to grayscale
	assertGrayscale(oldImg);
	assertGrayscale(newImg);

    double startingX = newX, startingY = newY;
	// Iterations of Gauss-Newton
	double mean = 0;
	for (int iter = 0; iter < params.maxIter; iter++) {
		// Print some info
		if (params.verbose > 0)
			std::cout << "ITER: " << iter << " | " << newX << " " << newY
					<< " Mean: " << mean << std::endl;

		if (params.verbose > 2)
		{
			// Visualize point
			cv::circle(drawImg, cv::Point2f((float)newX,(float)newY), 5,
					cv::Scalar(0, 0, 255));
			cv::imshow("Showing features", drawImg);
			cv::waitKey(10);
		}


		// If something is wrong -> skip the optimization
		if (std::isnan(newX) || std::isnan(newY) || std::isnan(mean)
				|| newX < params.halfPatchSize
				|| newX > newImg.cols - params.halfPatchSize
				|| newY < params.halfPatchSize
				|| newY > newImg.rows - params.halfPatchSize)
		{
			std::cout<<"InvHessian: " << std::endl << InvHessian << std::endl;

			std::cout<<"Out of image! (x,y) = (" << newX << ", " << newY << ")" << std::endl;

			if (params.verbose > 1)
			{
				getchar();
			}
			return false;
		}
		// Compute newPatch
		std::vector<double> newPatch = computePatch(newImg, newX, newY);

		// Evaluate patches
		Eigen::Vector3f tmpJ = Eigen::Vector3f::Zero();
		evaluatePatches(newPatch, oldPatch, tmpJ, gradientX, gradientY, mean);

		// Compute step
		Eigen::Vector3f increment = InvHessian * tmpJ;
		 increment[2] = 0;
		newX += increment[0];
		newY += increment[1];
		mean += increment[2];

		if (params.verbose > 1)
			std::cout<<"Moved by ("<<increment[0]<< " " << increment[1] << ")" << std::endl;

		// Ending condition
		if (increment[0] * increment[0] + increment[1] * increment[1]
				< params.minSqrtIncrement) {

			if (  (newX - startingX)*(newX - startingX) + (newY - startingY)*(newY - startingY) > params.halfPatchSize * params.halfPatchSize)
			{
				std::cout<<"Feature moved too far" << std::endl;
				return false;
			}

			std::cout<<"Feature ok!" << std::endl;
			if (params.verbose > 2)
			{
				getchar();
			}
			return true;
		}


	}

	std::cout<<"Maximum iterations reached" << std::endl;
	if (params.verbose > 2)
	{
		getchar();
	}

	return false;
}

int MatchingOnPatches::getPatchSize() {
	return params.patchSize;
}
int MatchingOnPatches::getHalfPatchSize() {
	return params.halfPatchSize;
}

void MatchingOnPatches::init(int _patchSize, int _maxIter, double _minSqrtIncrement,
		int _verbose) {
	params.patchSize = _patchSize;
	params.halfPatchSize = (params.patchSize - 1) / 2;
	params.verbose = _verbose;

	params.maxIter = _maxIter;
	params.minSqrtIncrement = _minSqrtIncrement;

	if (params.verbose > 0) {
		std::cout << "Patches size: " << params.patchSize << " maxIter: "
				<< params.maxIter << " minSqrtIncrement: "
				<< params.minSqrtIncrement << " verbose: " << params.verbose
				<< std::endl;
	}
}

inline void MatchingOnPatches::evaluatePatches(
		const std::vector<double> newPatch,
		const std::vector<double> oldPatch, Eigen::Vector3f & tmpJ,
		const std::vector<float> gradientX,
		const std::vector<float> gradientY,
		double mean) {
	for (int i = 0; i < params.patchSize * params.patchSize; i++) {
		if (params.verbose > 1)
		{
			std::cout << "Compare: " <<  newPatch[i] << " "
					<<  oldPatch[i] << " " << mean << " diff="
					<< newPatch[i] - oldPatch[i] + mean << std::endl;
		}
		float diff = (float)(newPatch[i] - oldPatch[i] + mean);
		tmpJ[0] = tmpJ[0] - diff * gradientX[i];
		tmpJ[1] = tmpJ[1] - diff * gradientY[i];
		tmpJ[2] = tmpJ[2] - diff;
	}
	if (params.verbose > 1)
		std::cout<< "Increments: " << tmpJ[0] << " " << tmpJ[1] << " " << tmpJ[2] <<std::endl;
}

inline void MatchingOnPatches::assertGrayscale(cv::Mat &img) {
	// Convert to grayscale
	if (img.channels() != 1) {
		cv::Mat imgTmp;
		cv::cvtColor(img, imgTmp, CV_RGB2GRAY);
		img = imgTmp;
	}
}

