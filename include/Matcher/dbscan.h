/** @file dbscan.h
 *
 * \brief The proposed clustering algorithm
 * \author Michal Nowicki
 *
 */
#ifndef _DBSCAN
#define _DBSCAN

#include "Defs/opencv.h"
//#include <opencv2/features2d.hpp>
#include <vector>

class DBScan {

public:
	// 	eps -> maximal distance to neighbour
	//	minPts -> minimal number of points to form a cluster
	DBScan(double eps = 10, int minPts = 2, int featuresFromCluster = 1);

	// Unsupervised clustering algorithm without the need to classify all samples
	void run(std::vector<cv::KeyPoint> & clusteringSet);

private:
	// Parameters of DBScan
	double eps;
	int minPts;
	int featuresFromCluster;

	// More structures to store data
	std::vector<std::vector<float> > dist;
	std::vector<bool> visited;
	std::vector<int> cluster;

	// expands cluster by checking the neighbour's neighbours
	void expandCluster(std::vector<int> neighbourList,
			int clusteringSetSize,
			int &C);

	int findingClusters(int clusteringSetSize);
};
#endif
