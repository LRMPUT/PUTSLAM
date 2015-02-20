/** @file dbscan.h
 *
 * \brief The proposed clustering algorithm
 * \author Michal Nowicki
 *
 */
#ifndef _DBSCAN
#define _DBSCAN

#include "opencv2/nonfree/features2d.hpp"
#include <vector>

class DBScan {
private:
	// expands cluster by checking the neighbour's neighbours
	void expandCluster(std::vector<int> neighbourList, bool *visited,
			int clusteringSetSize, double **dist, std::vector<int> & cluster,
			int & C);

public:
	double eps;
	int minPts;

	// 	eps -> maximal distance to neighbour
	//	minPts -> minimal number of points to form a cluster
	DBScan(double eps, int minPts);

	// Unsupervised clustering algorithm without the need to classify all samples
	void run(std::vector<cv::KeyPoint> & clusteringSet,
			std::vector<int> & cluster);
};
#endif
