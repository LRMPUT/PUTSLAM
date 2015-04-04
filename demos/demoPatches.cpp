#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <chrono>

#include "../include/Matcher/MatchingOnPatches.h"








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



	// Create matching on patches of size 9x9, verbose = 0
	MatchingOnPatches matchingOnPatches(9);

	// Compute old patch
	std::vector<uint8_t> oldPatch;
	oldPatch = matchingOnPatches.computePatch(dst, x, y);

	// Compute gradient
	std::vector<float> gradientX, gradientY;
	Eigen::Matrix3f InvHessian = Eigen::Matrix3f::Zero();
	matchingOnPatches.computeGradient(dst, x, y, InvHessian, gradientX, gradientY);


	// Optimize position of the feature
	matchingOnPatches.optimizeLocation(dst, oldPatch, dst, newX, newY, gradientX, gradientY, InvHessian);



	// Print final result
	std::cout<<std::endl<<"FINAL: " << newX << " " << newY <<  std::endl;

}
