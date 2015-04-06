#include "../include/Matcher/MatchingOnPatches.h"

MatchingOnPatches::MatchingOnPatches(int _patchSize, int _maxIter,
		float _minSqrtIncrement, int _verbose) {
	patchSize = _patchSize;
	halfPatchSize = (patchSize - 1) / 2;
	verbose = _verbose;

	maxIter = _maxIter;
	minSqrtIncrement = _minSqrtIncrement;
}

std::vector<uint8_t> MatchingOnPatches::computePatch(cv::Mat img,
		putslam::float_type x, putslam::float_type y) {
	std::vector<uint8_t> patch;

	// subpix precision
	const float xLeft = int(x), yLeft = int(y);
	const float xSub = x - xLeft, ySub = y - yLeft;

	// From wiki: http://upload.wikimedia.org/math/9/b/4/9b4e1064436ecccd069ea238b656c063.png
	const float topLeft = (1.0 - xSub) * (1.0 - ySub);
	const float topRight = xSub * (1.0 - ySub);
	const float bottomLeft = (1.0 - xSub) * ySub;
	const float bottomRight = xSub * ySub;

	// compute patch
	unsigned char *input = (unsigned char*) (img.data);

	// Speed-up considerations
	const int yEnd = y + halfPatchSize;
	const int xEnd = x + halfPatchSize;

	for (int i = y - halfPatchSize; i <= yEnd; i++) {
		for (int j = x - halfPatchSize; j <= xEnd; j++) {

			float value = topLeft * input[img.step * i + j]
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
	const int yEnd = y + halfPatchSize;
	const int xEnd = x + halfPatchSize;

	Eigen::Matrix3f Hessian = Eigen::Matrix3f::Zero();

	for (int i = y - halfPatchSize; i <= yEnd; i++) {
		for (int j = x - halfPatchSize; j <= xEnd; j++) {

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
	float mean = 0;
	for (int iter = 0; iter < maxIter; iter++) {
		// Print some info
		if (verbose > 0)
			std::cout << "ITER: " << iter << " | " << newX << " " << newY
					<< " Mean: " << mean << std::endl;

		// Compute newPatch
		std::vector<uint8_t> newPatch = computePatch(oldImg, newX, newY);

		// Evaluate patches
		Eigen::Vector3f tmpJ = Eigen::Vector3f::Zero();
		evaluatePatches(newPatch, oldPatch, tmpJ, gradientX, gradientY);

		// Compute step
		Eigen::Vector3f increment = InvHessian * tmpJ;
		newX += increment[0];
		newY += increment[1];
		mean += increment[2];

		if (std::isnan(newX) || std::isnan(newY) || std::isnan(mean)
				|| newX < halfPatchSize || newX > oldImg.cols - halfPatchSize
				|| newY < halfPatchSize || newY > oldImg.rows - halfPatchSize)
			return false;

		// Ending condition
		if (increment[0] * increment[0] + increment[1] * increment[1]
				< minSqrtIncrement) {
			return true;
		}
	}
	return false;
}

inline void MatchingOnPatches::evaluatePatches(
		const std::vector<uint8_t> newPatch,
		const std::vector<uint8_t> oldPatch, Eigen::Vector3f & tmpJ,
		const std::vector<float> gradientX,
		const std::vector<float> gradientY) {
	for (int i = 0; i < patchSize * patchSize; i++) {
		float diff = newPatch[i] - oldPatch[i]; // + mean;
		tmpJ[0] = tmpJ[0] - diff * gradientX[i];
		tmpJ[1] = tmpJ[1] - diff * gradientY[i];
		tmpJ[2] = tmpJ[2] - diff;
	}
}
