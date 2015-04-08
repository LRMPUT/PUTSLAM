#ifndef _PATCHES
#define _PATCHES

#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Eigen>

#include "../Defs/putslam_defs.h"

class MatchingOnPatches {

public:

	// Constructor. Takes:
	// 	-	patchSize	-> size of the used patches in pixels and equal in row and column size
	// 	-	maxIter		-> max iterations of Gauss-Newton optimization
	// 	-	minSqrtIncrement	-> Optimization threshold - lower step update than this value indicates convergence
	//  - 	verbose		-> verbose level (0 or 1)
	MatchingOnPatches(int _patchSize, int _maxIter = 20,
			float _minSqrtIncrement = 0.04, int _verbose = 0);

	// Computes the patch on image "img" at location (x,y)
	std::vector<uint8_t> computePatch(cv::Mat img, putslam::float_type x,
			putslam::float_type y);

	// Method used to compute the gradient on the old image for optimization purposes. Takes:
	// 	-	oldImg		-> old image used to compute old patch
	// 	-	x, y		-> location around we compute the gradients and hessian
	//	- 	gradientX, gradientY -> gradients of the patch on the old image
	//	-	InvHessian	-> inverse of hessian of the patch on the old image
	void computeGradient(cv::Mat img, putslam::float_type x,
			putslam::float_type y, Eigen::Matrix3f &InvHessian,
			std::vector<float> &gradientX, std::vector<float> &gradientY);

	// Gauss-Newton optimization used to find the new position of feature. Takes:
	// 	-	oldImg		-> old image used to compute old patch
	//	-	oldPatch	-> patch computed on the old patch
	//	-	newImg		-> new image on which we are looking for feature
	//	-	newX, newY	-> firstly the guess of feature location, used to return optimized location
	//	- 	gradientX, gradientY -> precomputed gradients of the patch on the old image
	//	-	InvHessian	-> precomputed inverse of hessian of the patch on the old image
	bool optimizeLocation(cv::Mat oldImg, std::vector<uint8_t> oldPatch,
			cv::Mat newImg, putslam::float_type &newX,
			putslam::float_type &newY, std::vector<float> gradientX,
			std::vector<float> gradientY, Eigen::Matrix3f &InvHessian);

	// Getters
	int getPatchSize();
	int getHalfPatchSize();

private:

	// Method used to compute the Jacobian of optimization
	inline void evaluatePatches(const std::vector<uint8_t> newPatch,
			const std::vector<uint8_t> oldPatch, Eigen::Vector3f & tmpJ,
			const std::vector<float> gradientX,
			const std::vector<float> gradientY);

	// Vebose level - 0 or 1
	int verbose;

	// Patch size in pixels and equal in row and column size
	int patchSize, halfPatchSize;

	// Max iterations of Gauss-Newton optimization
	int maxIter;

	// Optimization threshold - lower step update than this value indicates convergence
	float minSqrtIncrement;

};

#endif // _PATCHES
