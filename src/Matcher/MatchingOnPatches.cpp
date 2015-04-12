#include "../include/Matcher/MatchingOnPatches.h"

MatchingOnPatches::MatchingOnPatches(int _patchSize, int _maxIter,
		double _minSqrtIncrement, int _verbose) {
	init(_patchSize, _maxIter, _minSqrtIncrement, _verbose);
}

MatchingOnPatches::MatchingOnPatches(parameters _parameters) {
	init(_parameters.patchSize, _parameters.maxIter, _parameters.minSqrtIncrement, _parameters.verbose);
}

std::vector<uint8_t> MatchingOnPatches::computePatch(cv::Mat img,
		putslam::float_type x, putslam::float_type y) {
	std::vector<uint8_t> patch;

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
	const int yEnd = y + params.halfPatchSize;
	const int xEnd = x + params.halfPatchSize;

	for (int i = y - params.halfPatchSize; i <= yEnd; i++) {
		for (int j = x - params.halfPatchSize; j <= xEnd; j++) {

			uint8_t value = topLeft * input[img.step * i + j]
					+ topRight * input[img.step * (i) + j + 1]
					+ bottomLeft * input[img.step * (i + 1) + j]
					+ bottomRight * input[img.step * (i + 1) + j + 1];

			patch.push_back(value);
		}
	}

	return patch;
}

void MatchingOnPatches::computeGradient(cv::Mat img, putslam::float_type x,
		putslam::float_type y, Eigen::Matrix3f &InvHessian,
		std::vector<float> &gradientX, std::vector<float> &gradientY) {
	// compute patch
	unsigned char *input = (unsigned char*) (img.data);

	// Speed-up considerations
	const int yEnd = y + params.halfPatchSize;
	const int xEnd = x + params.halfPatchSize;

	Eigen::Matrix3f Hessian = Eigen::Matrix3f::Zero();

	for (int i = y - params.halfPatchSize; i <= yEnd; i++) {
		for (int j = x - params.halfPatchSize; j <= xEnd; j++) {

			Eigen::Vector3f J;
			J[0] =
					0.5
							* (input[img.step * i + j + 1]
									- input[img.step * i + j - 1]);
			J[1] = 0.5
					* (input[img.step * (i + 1) + j]
							- input[img.step * (i - 1) + j]);
			J[2] = 1.0;
			Hessian += J * J.transpose();

			gradientX.push_back(J[0]);
			gradientY.push_back(J[1]);
		}
	}
	InvHessian = Hessian.inverse();
}

bool MatchingOnPatches::optimizeLocation(cv::Mat oldImg,
		std::vector<uint8_t> oldPatch, cv::Mat newImg,
		putslam::float_type &newX, putslam::float_type &newY,
		std::vector<float> gradientX, std::vector<float> gradientY,
		Eigen::Matrix3f &InvHessian) {
	// Jacobian and hessian of old patch

	// Iterations of Gauss-Newton
	double mean = 0;
	for (int iter = 0; iter < params.maxIter; iter++) {
		// Print some info
		if (params.verbose > 0)
			std::cout << "ITER: " << iter << " | " << newX << " " << newY
					<< " Mean: " << mean << std::endl;

		// If something is wrong -> skip the optimization
		if (std::isnan(newX) || std::isnan(newY) || std::isnan(mean)
				|| newX < params.halfPatchSize
				|| newX > newImg.cols - params.halfPatchSize
				|| newY < params.halfPatchSize
				|| newY > newImg.rows - params.halfPatchSize)
			return false;

		// Compute newPatch
		std::vector<uint8_t> newPatch = computePatch(newImg, newX, newY);

		// Evaluate patches
		Eigen::Vector3f tmpJ = Eigen::Vector3f::Zero();
		evaluatePatches(newPatch, oldPatch, tmpJ, gradientX, gradientY);

		// Compute step
		Eigen::Vector3f increment = InvHessian * tmpJ;
		newX += increment[0];
		newY += increment[1];
		mean += increment[2];

		// Ending condition
		if (increment[0] * increment[0] + increment[1] * increment[1]
				< params.minSqrtIncrement) {
			return true;
		}
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
		const std::vector<uint8_t> newPatch,
		const std::vector<uint8_t> oldPatch, Eigen::Vector3f & tmpJ,
		const std::vector<float> gradientX,
		const std::vector<float> gradientY) {
	for (int i = 0; i < params.patchSize * params.patchSize; i++) {
		double diff = newPatch[i] - oldPatch[i]; // + mean;
		tmpJ[0] = tmpJ[0] - diff * gradientX[i];
		tmpJ[1] = tmpJ[1] - diff * gradientY[i];
		tmpJ[2] = tmpJ[2] - diff;
	}
}
