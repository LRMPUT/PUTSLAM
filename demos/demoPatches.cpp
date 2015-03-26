#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <chrono>


std::vector<uint8_t> computePatch (cv::Mat img, float x, float y, int patchSize) {
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
	const int halfPatchSize = (patchSize - 1) / 2;
	const int yEnd = y + halfPatchSize;
	const int xEnd = x + halfPatchSize;

	for (int i = y - halfPatchSize; i <= yEnd; i++) {
		for (int j = x - halfPatchSize; j <= xEnd;
				j++) {

			float value = topLeft * input[img.step * i + j]
					+ topRight * input[img.step * (i) + j + 1]
					+ bottomLeft * input[img.step * (i+1) + j]
					+ bottomRight * input[img.step * (i+1) + j + 1];

			patch.push_back(value);
		}
	}

	return patch;
}


void computeGradient(cv::Mat img, float x, float y, int patchSize,
		Eigen::Matrix3f &InvHessian, std::vector<float> &gradientX,
		std::vector<float> &gradientY) {
	// compute patch
		unsigned char *input = (unsigned char*) (img.data);

		// Speed-up considerations
		const int halfPatchSize = (patchSize - 1) / 2;
		const int yEnd = y + halfPatchSize;
		const int xEnd = x + halfPatchSize;

		Eigen::Matrix3f Hessian = Eigen::Matrix3f::Zero();

		for (int i = y - halfPatchSize; i <= yEnd; i++) {
			for (int j = x - halfPatchSize; j <= xEnd; j++) {

				Eigen::Vector3f J;
				J[0] = 0.5
						* (input[img.step * i + j +1 ]
								- input[img.step * i + j -1]);
				J[1] = 0.5
						* (input[img.step * (i+1) + j]
								- input[img.step * (i-1) + j]);
				J[2] = 1.0;
				Hessian += J * J.transpose();

				gradientX.push_back(J[0]);
				gradientY.push_back(J[1]);
			}
		}
		InvHessian = Hessian.inverse();
}

inline void evaluatePatches(const std::vector<uint8_t> newPatch,
		const std::vector<uint8_t> oldPatch, Eigen::Vector3f & tmpJ,
		const std::vector<float> gradientX,
		const std::vector<float> gradientY, const int patchSize) {
	for (int i = 0; i < patchSize * patchSize; i++) {
		float diff = newPatch[i] - oldPatch[i]; // + mean;
		tmpJ[0] = tmpJ[0] - diff * gradientX[i];
		tmpJ[1] = tmpJ[1] - diff * gradientY[i];
		tmpJ[2] = tmpJ[2] - diff;
	}
}

int main()
{
	// Just reading random image
	cv::Mat img = cv::imread("0.png", CV_LOAD_IMAGE_COLOR ), dst;

	// Just use gray image
	cv::cvtColor(img, dst, CV_RGB2GRAY);

	// Select random feature
	float x = 150.3f, y = 253.1f;

	// Visualize point
//	cv::circle(img, cv::Point2f(x,y), 5, cv::Scalar(0, 0 ,255));
//	cv::imshow("Showing features", img);
//	cv::waitKey(5000);

	// Distruption to point
	float newX = x + (rand()%100)/20.0, newY = y + (rand()%100)/20.0;
	std::cout<<"x, y = " << x << " " << y << std::endl;
	std::cout<<"newX, newY = " << newX << " " << newY <<std::endl;

	// Patch size 9x9
	const int patchSize = 9;

	// Compute old patch
	std::vector<uint8_t> oldPatch;
	oldPatch = computePatch(dst, x, y, patchSize);

	// Jacobian and hessian of old patch
	std::vector<float> gradientX, gradientY;
	Eigen::Matrix3f InvHessian = Eigen::Matrix3f::Zero();
	computeGradient(dst, x, y, patchSize, InvHessian, gradientX, gradientY);

	// Iterations of Gauss-Newton
	float mean = 0;
	int maxIter = 20;
	const float minSqrtIncrement = 0.04;
	for (int iter = 0; iter < maxIter ; iter++)
	{
		// Print some info
		std::cout<<"ITER: " << iter << " | " << newX << " " << newY <<  " Mean: " << mean << std::endl;

		// Compute newPatch
		std::vector<uint8_t> newPatch = computePatch (dst, newX, newY, patchSize);

		// Evaluate patches
		Eigen::Vector3f tmpJ = Eigen::Vector3f::Zero();
		evaluatePatches(newPatch, oldPatch, tmpJ, gradientX, gradientY,
				patchSize);

		// Compute step
		Eigen::Vector3f increment = InvHessian * tmpJ;
		newX += increment[0];
		newY += increment[1];
		mean += increment[2];

		// Ending condition
		if (increment[0]*increment[0] + increment[1] * increment[1] < minSqrtIncrement) {
			break;
		}
	}

	// Print final result
	std::cout<<std::endl<<"FINAL: " << newX << " " << newY <<  " Mean: " << mean << std::endl;

}
