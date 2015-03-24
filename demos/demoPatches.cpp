#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Eigen>

int main()
{
	// Just reading random image
	cv::Mat img = cv::imread("0.png", CV_LOAD_IMAGE_COLOR ), dst;

	// Just use gray image
	cv::cvtColor(img, dst, CV_RGB2GRAY);

	// Select random feature
	float x = 150.0f, y = 253.0f;

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

	// Old Patch -> non-effective way
	std::vector<uint8_t> oldPatch;
	for (int i= y - (patchSize-1)/2 ; i<= y + (patchSize-1)/2 ; i++ )
		for (int j= x - (patchSize-1)/2 ; j<= x + (patchSize-1)/2 ; j++ )
			oldPatch.push_back(dst.at<uint8_t>(cv::Point2f(j,i)) );

	// Jacobian and hessian of old patch
	Eigen::Matrix3f Hessian = Eigen::Matrix3f::Zero();
	std::vector<float> gradientX, gradientY;
	for (int i = y - (patchSize - 1) / 2; i <= y + (patchSize - 1) / 2; i++)
	{
		for (int j = x - (patchSize - 1) / 2; j <= x + (patchSize - 1) / 2; j++)
		{
			Eigen::Vector3f J;
			J[0] = 0.5 * (dst.at<uint8_t>(cv::Point2f(j+1, i)) - dst.at<uint8_t>(cv::Point2f(j-1, i)));
			J[1] = 0.5 * (dst.at<uint8_t>(cv::Point2f(j, i+1)) - dst.at<uint8_t>(cv::Point2f(j, i-1)));
			J[2] = 1.0;
			//std::cout<<"J" << J << std::endl;
			Hessian += J*J.transpose();

			gradientX.push_back(J[0]);
			gradientY.push_back(J[1]);

			oldPatch.push_back(dst.at<uint8_t>(cv::Point2f(j, i)));
		}
	}
	Eigen::Matrix3f InvHessian = Hessian.inverse();

//	std::cout<<"Hessian: " << std::endl << Hessian << std::endl;
//	std::cout<<"InvHessian: " << std::endl << InvHessian << std::endl;

	// Iterations of Gauss-Newton
	const int iterCount = 15;
	float mean = 0;
	for (int iter = 0 ; iter < iterCount; iter++)
	{
		// Print some info
		std::cout<<"ITER: " << iter << " | " << newX << " " << newY <<  " Mean: " << mean << std::endl;

		// tmps
		float xLeft = int(newX), yLeft = int(newY);
		float xSub = newX - xLeft, ySub = newY - yLeft;

		// From wiki: http://upload.wikimedia.org/math/9/b/4/9b4e1064436ecccd069ea238b656c063.png
		float topLeft = (1.0 - xSub) * (1.0 - ySub);
		float topRight = xSub * (1.0 - ySub);
		float bottomLeft = (1.0 - xSub) * ySub;
		float bottomRight = xSub * ySub;

		std::cout << "\ttopLeft: " << (1.0 - xSub) << " " << (1.0 - ySub)
				<< std::endl;
		std::cout << "\ttopRight: " << xSub << " " << (1.0 - ySub) << std::endl;
		std::cout << "\tbottomLeft: " << (1.0 - xSub) << " " << ySub
				<< std::endl;
		std::cout << "\tbottomRight: " << xSub << " " << ySub << std::endl;

		// New Patch
		std::vector<uint8_t> newPatch;
		for (int i = newY - (patchSize - 1) / 2; i <= newY + (patchSize - 1) / 2; i++)
		{
			for (int j = newX - (patchSize - 1) / 2; j <= newX + (patchSize - 1) / 2; j++)
			{
				float value = topLeft * dst.at<uint8_t>(cv::Point2f(j,i)) + topRight * dst.at<uint8_t>(cv::Point2f(j+1, i))
						+ bottomLeft * dst.at<uint8_t>(cv::Point2f(j, i+1)) + bottomRight * dst.at<uint8_t>(cv::Point2f(j+1,i+1));
				newPatch.push_back(value);
			}
		}

		// Evaluate patches
		Eigen::Vector3f tmpJ = Eigen::Vector3f::Zero();
		for (int i=0;i<patchSize*patchSize;i++) {
			float diff = newPatch[i] - oldPatch[i];// + mean;
			tmpJ[0] = tmpJ[0] - diff * gradientX[i];
			tmpJ[1] = tmpJ[1] - diff * gradientY[i];
			tmpJ[2] = tmpJ[2] - diff;
		}

		// Compute step
		Eigen::Vector3f increment = InvHessian * tmpJ;
		newX += increment[0];
		newY += increment[1];
		mean += increment[2];
	}
}
