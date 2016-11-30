#ifndef _PATCHES
#define _PATCHES

#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "Defs/eigen3.h"

#include "Defs/putslam_defs.h"

class MatchingOnPatches {

public:
	struct parameters {
		// Vebose level - 0 or 1
		int verbose;

		// Perform warping
		bool warping;

		// Patch size in pixels and equal in row and column size
		int patchSize, halfPatchSize;

		// Max iterations of Gauss-Newton optimization
		int maxIter;

		// Optimization threshold - lower step update than this value indicates convergence
		double minSqrtIncrement;
	};

	// Constructor. Takes:
	// 	-	patchSize	-> size of the used patches in pixels and equal in row and column size
	// 	-	maxIter		-> max iterations of Gauss-Newton optimization
	// 	-	minSqrtIncrement	-> Optimization threshold - lower step update than this value indicates convergence
	//  - 	verbose		-> verbose level (0 or 1)
	MatchingOnPatches(int _patchSize, int _maxIter = 20,
			double _minSqrtIncrement = 0.04, int _verbose = 0);

	// Constructor based on parameters
	MatchingOnPatches(parameters _parameters);

	// Computes the patch on image "img" at location (x,y)
    std::vector<double> computePatch(cv::Mat img, double x,
            double y);

	// Method used to compute the gradient on the old image for optimization purposes. Takes:
	// 	-	oldImg		-> old image used to compute old patch
	// 	-	x, y		-> location around we compute the gradients and hessian
	//	- 	gradientX, gradientY -> gradients of the patch on the old image
	//	-	InvHessian	-> inverse of hessian of the patch on the old image
    void computeGradient(cv::Mat img, double x,
            double y, Eigen::Matrix3f &InvHessian,
			std::vector<float> &gradientX, std::vector<float> &gradientY);

	// Gauss-Newton optimization used to find the new position of feature. Takes:
	// 	-	oldImg		-> old image used to compute old patch
	//	-	oldPatch	-> patch computed on the old patch
	//	-	newImg		-> new image on which we are looking for feature
	//	-	newX, newY	-> firstly the guess of feature location, used to return optimized location
	//	- 	gradientX, gradientY -> precomputed gradients of the patch on the old image
	//	-	InvHessian	-> precomputed inverse of hessian of the patch on the old image
	bool optimizeLocation(cv::Mat oldImg, std::vector<double> oldPatch,
            cv::Mat newImg, double &newX,
            double &newY, std::vector<float> gradientX,
			std::vector<float> gradientY, Eigen::Matrix3f &InvHessian);

	// Getters
	int getPatchSize();
	int getHalfPatchSize();

private:

	// Parameters
	parameters params;

	// Methods used to initialize parameters
	void init(int _patchSize, int _maxIter, double _minSqrtIncrement,
			int _verbose);

	// Method used to compute the Jacobian of optimization
	inline void evaluatePatches(const std::vector<double> newPatch,
			const std::vector<double> oldPatch, Eigen::Vector3f & tmpJ,
			const std::vector<float> gradientX,
			const std::vector<float> gradientY, double mean = 0.0);

	// Method used to check that image is grayscale
	inline void assertGrayscale(cv::Mat &img);
};

#endif // _PATCHES
